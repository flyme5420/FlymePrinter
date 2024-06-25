#ifndef SERIALREADER_H
#define SERIALREADER_H

#include <string>
#include <map>
#include <thread>
#include <mutex>
#include <functional>
#include <atomic>
#include <condition_variable>
#include "Reactor.h"
#include "MessageParser.h"

class SerialReader {
public:
    SerialReader(Reactor* reactor, const std::string& warn_prefix = "");
    ~SerialReader();

    bool startSession(int serial_dev_fd, char serial_fd_type = 'u', int client_id = 0);
    void connectPipe(const std::string& filename);
    void connectUart(const std::string& serialport, int baud, bool rts = true);
    void connectFile(int debugoutput_fd, const std::string& dictionary, bool pace = false);
    void setClockEst(float freq, double conv_time, double conv_clock, double last_clock);
    void disconnect();
    std::string stats(double eventtime);
    Reactor* getReactor();
    MessageParser* getMsgParser();
    int getSerialQueue();
    int getDefaultCommandQueue();
    void registerResponse(std::function<void(const Params&)> callback, const std::string& name, int oid = -1);
    void send(const std::string& msg, int minclock = 0, int reqclock = 0);
    Params sendWithResponse(const std::string& msg, const std::string& response);

private:
    void bgThread();
    void error(const std::string& msg);
    std::string getIdentifyData(double eventtime);

    Reactor* reactor;
    std::string warn_prefix;
    int serial_dev;
    MessageParser msgparser;
    Chelper chelper;
    int serialqueue;
    int default_cmd_queue;
    char stats_buf[4096];
    std::thread background_thread;
    std::mutex lock;
    std::map<std::pair<std::string, int>, std::function<void(const Params&)>> handlers;
    std::atomic<int> last_notify_id;
    std::map<int, Completion*> pending_notifications;
};

class SerialRetryCommand {
public:
    SerialRetryCommand(SerialReader* serial, const std::string& name, int oid = -1);
    Params getResponse(const std::vector<std::string>& cmds, int cmd_queue, int minclock = 0, int reqclock = 0);

private:
    void handleCallback(const Params& params);

    SerialReader* serial;
    std::string name;
    int oid;
    Params last_params;
    std::condition_variable cv;
    std::mutex mtx;
};

#endif // SERIALREADER_H
