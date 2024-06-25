#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include "serialhdl.h"

SerialReader::SerialReader(Reactor* reactor, const std::string& warn_prefix)
    : reactor(reactor), warn_prefix(warn_prefix), serial_dev(-1), 
      serialqueue(-1), default_cmd_queue(alloc_command_queue()), 
      last_notify_id(0) {
    registerResponse([this](const Params& params) { this->handleUnknownInit(params); }, "#unknown");
    registerResponse([this](const Params& params) { this->handleOutput(params); }, "#output");
}

SerialReader::~SerialReader() {
    disconnect();
}

bool SerialReader::startSession(int serial_dev_fd, char serial_fd_type, int client_id) {
    serial_dev = serial_dev_fd;
    serialqueue = serialqueue_alloc(serial_dev_fd, serial_fd_type, client_id);

    background_thread = std::thread(&SerialReader::bgThread, this);
    background_thread.detach();

    auto identify_data = getIdentifyData(reactor->monotonic() + 5.0);
    if (identify_data.empty()) {
        std::cerr << warn_prefix << "Timeout on connect" << std::endl;
        disconnect();
        return false;
    }

    msgparser.process_identify(identify_data);

    double wire_freq = msgparser.get_constant_float(serial_fd_type == 'c' ? "CANBUS_FREQUENCY" : "SERIAL_BAUD", 0.0);
    if (wire_freq != 0.0) {
        serialqueue_set_wire_frequency(serialqueue, wire_freq);
    }

    int receive_window = msgparser.get_constant_int("RECEIVE_WINDOW", 0);
    if (receive_window != 0) {
        serialqueue_set_receive_window(serialqueue, receive_window);
    }

    return true;
}

void SerialReader::bgThread() {
    while (true) {
        auto response = serialqueue_pull(serialqueue);
        if (response.len < 0) break;

        if (response.notify_id != 0) {
            Params params;
            params["#sent_time"] = response.sent_time;
            params["#receive_time"] = response.receive_time;

            auto completion = pending_notifications[response.notify_id];
            pending_notifications.erase(response.notify_id);
            reactor->async_complete(completion, params);
            continue;
        }

        Params params = msgparser.parse(std::string(response.msg, response.len));
        params["#sent_time"] = response.sent_time;
        params["#receive_time"] = response.receive_time;

        std::lock_guard<std::mutex> guard(lock);
        auto handler = handlers.find({params["#name"], params.get("oid", -1)});
        if (handler != handlers.end()) {
            handler->second(params);
        } else {
            handle_default(params);
        }
    }
}

void SerialReader::error(const std::string& msg) {
    throw std::runtime_error(warn_prefix + msg);
}

std::string SerialReader::getIdentifyData(double eventtime) {
    std::string identify_data;
    while (true) {
        std::string msg = "identify offset=" + std::to_string(identify_data.size()) + " count=40";
        try {
            Params params = sendWithResponse(msg, "identify_response");
            if (params["offset"] == std::to_string(identify_data.size())) {
                std::string msgdata = params["data"];
                if (msgdata.empty()) {
                    return identify_data;
                }
                identify_data += msgdata;
            }
        } catch (const std::exception& e) {
            std::cerr << warn_prefix << "Wait for identify_response: " << e.what() << std::endl;
            return "";
        }
    }
}

void SerialReader::connectPipe(const std::string& filename) {
    std::cerr << warn_prefix << "Starting connect" << std::endl;
    double start_time = reactor->monotonic();
    while (true) {
        if (reactor->monotonic() > start_time + 90.0) {
            error("Unable to connect");
        }
        int fd = open(filename.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1) {
            std::cerr << warn_prefix << "Unable to open port: " << strerror(errno) << std::endl;
            reactor->pause(reactor->monotonic() + 5.0);
            continue;
        }
        if (startSession(fd, 'u', 0)) {
            break;
        }
    }
}

void SerialReader::connectUart(const std::string& serialport, int baud, bool rts) {
    std::cerr << warn_prefix << "Starting serial connect" << std::endl;
    double start_time = reactor->monotonic();
    while (true) {
        if (reactor->monotonic() > start_time + 90.0) {
            error("Unable to connect");
        }
        try {
            serial::Serial serial_dev(serialport, baud, serial::Timeout::simpleTimeout(0));
            serial_dev.setRTS(rts);
            serial_dev.open();
            stk500v2_leave(&serial_dev, reactor);
            if (startSession(serial_dev.fd())) {
                break;
            }
        } catch (const std::exception& e) {
            std::cerr << warn_prefix << "Unable to open serial port: " << e.what() << std::endl;
            reactor->pause(reactor->monotonic() + 5.0);
        }
    }
}

void SerialReader::connectFile(int debugoutput, const std::string& dictionary, bool pace) {
    serial_dev = debugoutput;
    msgparser.process_identify(dictionary, false);
    serialqueue = serialqueue_alloc(serial_dev, 'f', 0);
}

void SerialReader::setClockEst(double freq, double conv_time, double conv_clock, double last_clock) {
    serialqueue_set_clock_est(serialqueue, freq, conv_time, conv_clock, last_clock);
}

void SerialReader::disconnect() {
    if (serialqueue != -1) {
        serialqueue_exit(serialqueue);
        if (background_thread.joinable()) {
            background_thread.join();
        }
        serialqueue = -1;
    }
    if (serial_dev != -1) {
        close(serial_dev);
        serial_dev = -1;
    }
    for (auto& [id, pn] : pending_notifications) {
        pn.complete(nullptr);
    }
    pending_notifications.clear();
}

std::string SerialReader::stats(double eventtime) {
    if (serialqueue == -1) {
        return "";
    }
    char stats_buf[4096];
    serialqueue_get_stats(serialqueue, stats_buf, sizeof(stats_buf));
    return std::string(stats_buf);
}

Reactor* SerialReader::getReactor() {
    return reactor;
}

MessageParser* SerialReader::getMsgParser() {
    return &msgparser;
}

int SerialReader::getSerialQueue() {
    return serialqueue;
}

int SerialReader::getDefaultCommandQueue() {
    return default_cmd_queue;
}

void SerialReader::registerResponse(std::function<void(const Params&)> callback, const std::string& name, int oid) {
    std::lock_guard<std::mutex> guard(lock);
    if (callback == nullptr) {
        handlers.erase({name, oid});
    } else {
        handlers[{name, oid}] = callback;
    }
}

void SerialReader::send(const std::string& msg, int minclock, int reqclock) {
    auto cmd = msgparser.create_command(msg);
    serialqueue_send(serialqueue, default_cmd_queue, cmd.c_str(), cmd.size(), minclock, reqclock, 0);
}

Params SerialReader::sendWithResponse(const std::string& msg, const std::string& response) {
    auto cmd = msgparser.create_command(msg);
    SerialRetryCommand src(this, response);
    return src.getResponse({cmd}, default_cmd_queue);
}

int SerialReader::alloc_command_queue() {
    return serialqueue_alloc_commandqueue();
}

void SerialReader::rawSend(const std::string& cmd, int minclock, int reqclock, int cmd_queue) {
    serialqueue_send(serialqueue, cmd_queue, cmd.c_str(), cmd.size(), minclock, reqclock, 0);
}

Params SerialReader::rawSendWaitAck(const std::string& cmd, int minclock, int reqclock, int cmd_queue) {
    last_notify_id++;
    int nid = last_notify_id;
    auto completion = reactor->completion();
    pending_notifications[nid] = completion;
    serialqueue_send(serialqueue, cmd_queue, cmd.c_str(), cmd.size(), minclock, reqclock, nid);
    auto params = completion.wait();
    if (!params) {
        error("Serial connection closed");
    }
    return params.value();
}

SerialRetryCommand::SerialRetryCommand(SerialReader* serial, const std::string& name, int oid)
    : serial(serial), name(name), oid(oid) {
    serial->registerResponse([this](const Params& params) { handleCallback(params); }, name, oid);
}

Params SerialRetryCommand::getResponse(const std::vector<std::string>& cmds, int cmd_queue, int minclock, int reqclock) {
    int retries = 5;
    double retry_delay = 0.010;
    while (true) {
        for (size_t i = 0; i < cmds.size() - 1; ++i) {
            serial->rawSend(cmds[i], minclock, reqclock, cmd_queue);
        }
        serial->rawSendWaitAck(cmds.back(), minclock, reqclock, cmd_queue);
        if (last_params) {
            serial->registerResponse(nullptr, name, oid);
            return last_params.value();
        }
        if (retries <= 0) {
            serial->registerResponse(nullptr, name, oid);
            throw std::runtime_error("Unable to obtain '" + name + "' response");
        }
        reactor->pause(reactor->monotonic() + retry_delay);
        retries--;
        retry_delay *= 2;
    }
}

void SerialRetryCommand::handleCallback(const Params& params) {
    last_params = params;
}
