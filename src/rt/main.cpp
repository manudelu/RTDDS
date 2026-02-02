#include "rt_sockets.hpp"
#include <pthread.h>
#include <signal.h>
#include <iostream>

extern void* client(void*);
extern void* server(void*);

constexpr int NUM_THREADS = 2;
pthread_t threads[NUM_THREADS];

struct ThreadInfo {
    void* (*func)(void*);
    int policy;
    int priority;
};

int main() {
    // Block signals for main thread
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, nullptr);

    ThreadInfo thread_info[NUM_THREADS] = {
        { server, SCHED_FIFO, 71 },
        { client, SCHED_FIFO, 70 },
    };

    // Create RT threads
    for(int i=0; i<NUM_THREADS; ++i) {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, thread_info[i].policy);
        struct sched_param param{thread_info[i].priority};
        pthread_attr_setschedparam(&attr, &param);

        if(pthread_create(&threads[i], &attr, thread_info[i].func, nullptr))
            error_handler("pthread_create");
    }

    std::cout << "RT threads started. Press Ctrl+C to stop." << std::endl;

    // Wait for signal
    int sig;
    __STD(sigwait(&set, &sig));

    std::cout << "\nShutting down..." << std::endl;

    // Cancel RT threads
    for(int i=0; i<NUM_THREADS; ++i)
        pthread_cancel(threads[i]);

    // Wait for threads to finish
    for(int i=0; i<NUM_THREADS; ++i)
        pthread_join(threads[i], nullptr);

    return 0;
}