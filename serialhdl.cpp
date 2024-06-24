#if 0
#include <iostream>
#include <thread>
#include <mutex>
#include <map>
#include <vector>
#include <condition_variable>
#include <serial/serial.h> // 需要安装serial库
#include "msgproto.h" // 假设该库存在并具有相应的功能
#include "chelper.h" // 假设该库存在并具有相应的功能
#include "reactor.h" // 假设该库存在并具有相应的功能

class SerialReader {
public:
    SerialReader(Reactor& reactor, const std::string& warn_prefix = "")
        : reactor(reactor), warn_prefix(warn_prefix), serial_dev(nullptr),
          serialqueue(nullptr), default_cmd_queue(alloc_command_queue()),
          background_thread(nullptr), last_notify_id(0) {
        // 初始化 msgparser
        msgparser = new MessageParser(warn_prefix);
        register_response(std::bind(&SerialReader::_handle_unknown_init, this, std::placeholders::_1), "#unknown");
        register_response(std::bind(&SerialReader::handle_output, this, std::placeholders::_1), "#output");
    }

    ~SerialReader() {
        disconnect();
        delete msgparser;
    }

    void startSession(int fd, const std::string& type = "u", int client_id = 0) {
        serialqueue = serialqueue_alloc(fd, type.c_str(), client_id);
        background_thread = new std::thread(&SerialReader::bgThread, this);
        background_thread->detach();
    }

    void send(const std::string& msg, int minclock = 0, int reqclock = 0) {
        auto cmd = msgparser->createCommand(msg);
        rawSend(cmd, minclock, reqclock, default_cmd_queue);
    }

    void disconnect() {
        if (serialqueue != nullptr) {
            serialqueue_exit(serialqueue);
            if (background_thread != nullptr) {
                background_thread->join();
                delete background_thread;
            }
            serialqueue_free(serialqueue);
            serialqueue = nullptr;
        }
        if (serial_dev != nullptr) {
            serial_dev->close();
            delete serial_dev;
            serial_dev = nullptr;
        }
        for (auto& pn : pending_notifications) {
            pn.second->complete(nullptr);
        }
        pending_notifications.clear();
    }

    std::string getStats() {
        if (serialqueue == nullptr) return "";
        char stats_buf[4096];
        serialqueue_get_stats(serialqueue, stats_buf, sizeof(stats_buf));
        return std::string(stats_buf);
    }

    void connectUART(const std::string& serialport, unsigned long baud, bool rts = true) {
        std::cout << warn_prefix << "Starting serial connect" << std::endl;
        auto start_time = reactor.monotonic();
        while (true) {
            if (reactor.monotonic() > start_time + 90.) {
                throw std::runtime_error(warn_prefix + "Unable to connect");
            }
            try {
                serial_dev = new serial::Serial(serialport, baud, serial::Timeout::simpleTimeout(1000));
                serial_dev->setRTS(rts);
            } catch (const serial::IOException& e) {
                std::cerr << warn_prefix << "Unable to open serial port: " << e.what() << std::endl;
                reactor.pause(reactor.monotonic() + 5.);
                continue;
            }
            stk500v2_leave(*serial_dev, reactor); // 假设存在此函数
            bool ret = startSession(serial_dev->getNativeHandle());
            if (ret) {
                break;
            }
        }
    }

private:
    Reactor& reactor;
    std::string warn_prefix;
    serial::Serial* serial_dev;
    void* serialqueue;
    MessageParser* msgparser;
    void* default_cmd_queue;
    std::mutex lock;
    std::thread* background_thread;
    int last_notify_id;
    std::map<int, Completion*> pending_notifications;

    void bgThread() {
        pull_queue_message response;
        while (true) {
            serialqueue_pull(serialqueue, &response);
            if (response.len < 0) break;

            if (response.notify_id) {
                auto params = std::map<std::string, int64_t>{{"#sent_time", response.sent_time}, {"#receive_time", response.receive_time}};
                auto completion = pending_notifications[response.notify_id];
                pending_notifications.erase(response.notify_id);
                reactor.asyncComplete(completion, params);
                continue;
            }

            auto params = msgparser->parse(std::string(response.msg, response.len));
            params["#sent_time"] = response.sent_time;
            params["#receive_time"] = response.receive_time;

            std::lock_guard<std::mutex> guard(lock);
            auto hdl = handlers.find({params["#name"], params.get("oid")});
            if (hdl != handlers.end()) {
                hdl->second(params);
            } else {
                handleDefault(params);
            }
        }
    }

    void rawSend(const std::string& cmd, int minclock, int reqclock, void* cmd_queue) {
        serialqueue_send(serialqueue, cmd_queue, cmd.c_str(), cmd.size(), minclock, reqclock, 0);
    }

    void* alloc_command_queue() {
        return serialqueue_alloc_commandqueue();
    }

    void handleDefault(const std::map<std::string, int>& params) {
        std::cerr << warn_prefix << " got unknown message: " << params.at("#msgid") << std::endl;
    }

    void register_response(void (*callback)(const std::map<std::string, int>&), const std::string& name, int oid = 0) {
        std::lock_guard<std::mutex> guard(lock);
        handlers[{name, oid}] = callback;
    }

    std::map<std::pair<std::string, int>, void (*)(const std::map<std::string, int>&)> handlers;
};

class SerialRetryCommand {
public:
    SerialRetryCommand(SerialReader& serial, const std::string& name, int oid = 0)
        : serial(serial), name(name), oid(oid), last_params(nullptr) {
        serial.register_response(std::bind(&SerialRetryCommand::handleCallback, this, std::placeholders::_1), name, oid);
    }

    std::map<std::string, int> getResponse(const std::vector<std::string>& cmds, void* cmd_queue, int minclock = 0, int reqclock = 0) {
        int retries = 5;
        double retry_delay = .010;
        while (true) {
            for (const auto& cmd : cmds) {
                serial.rawSend(cmd, minclock, reqclock, cmd_queue);
            }
            serial.rawSendWaitAck(cmds.back(), minclock, reqclock, cmd_queue);
            if (last_params) {
                serial.register_response(nullptr, name, oid);
                return *last_params;
            }
            if (retries <= 0) {
                serial.register_response(nullptr, name, oid);
                throw std::runtime_error("Unable to obtain '" + name + "' response");
            }
            reactor.pause(reactor.monotonic() + retry_delay);
            retries -= 1;
            retry_delay *= 2.;
        }
    }

private:
    SerialReader& serial;
    std::string name;
    int oid;
    std::map<std::string, int>* last_params;

    void handleCallback(const std::map<std::string, int>& params) {
        last_params = new std::map<std::string, int>(params);
    }
};

int main() {
    // 示例主函数
    Reactor reactor;
    SerialReader reader(reactor, "WarningPrefix");
    reader.connectUART("/dev/ttyUSB0", 9600);

    // 读取和处理数据，通常放在主循环中
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << reader.getStats() << std::endl;
    }

    return 0;
}
#endif