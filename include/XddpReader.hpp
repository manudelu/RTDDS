#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <errno.h>

template <typename T>
class XddpReader
{
public:
    explicit XddpReader(std::string port_label)
        : port_label_(std::move(port_label))
        , fd_(-1)
    {
        snprintf(device_path_, sizeof(device_path_),
                 "/proc/xenomai/registry/rtipc/xddp/%s",
                 port_label_.c_str());
    }

    ~XddpReader()
    {
        close_device();
    }

    bool connect(std::atomic<bool>& running)
    {
        std::cout << "Waiting for XDDP port '" << port_label_ << "'..." << std::endl;

        while (running) {
            fd_ = open(device_path_, O_RDONLY);
            if (fd_ >= 0) {
                std::cout << "Connected to XDDP device!" << std::endl;
                return true;
            }

            if (errno == ENOENT) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            } else if (errno != EINTR) {
                std::cerr << "XDDP open error: "
                          << strerror(errno) << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        return false;
    }

    bool read(T& msg)
    {
        int ret = ::read(fd_, &msg, sizeof(T));

        if (ret == sizeof(T)) {
            return true;
        }

        if (ret < 0 && errno == EINTR) {
            return false;
        }

        reconnect();
        return false;
    }

    bool connected() const
    {
        return fd_ >= 0;
    }

private:
    void reconnect()
    {
        close_device();
        fd_ = open(device_path_, O_RDONLY);
    }

    void close_device()
    {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

private:
    std::string port_label_;
    char device_path_[256];
    int fd_;
};
