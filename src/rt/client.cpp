#include "rt_sockets.hpp"
#include "JointStateMsg.hpp"
#include "TrajectoryGenerator.hpp"
#include <vector>
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

std::vector<double> pos(N_JOINTS), vel(N_JOINTS); 

void* client(void* arg) {
    struct rt_joint_state_msg msg;
    uint64_t seq = 0;

    TrajectoryGenerator traj_gen(robot, end_pos, 5.0, TrajectoryType::Quintic);

    int s = create_rt_socket(IDDP_PORT_LABEL, true, RtSocketRole::Client);

    struct timespec sleep_ts = {0, 50'000'000}; // 20Hz
    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    struct timespec ts_now;
    double t;

    for(;;) {
        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        t = (ts_now.tv_sec - t0.tv_sec) + (ts_now.tv_nsec - t0.tv_nsec)/1e9;

        msg.seq = seq++;
        msg.timestamp_ns = ts_now.tv_sec * 1'000'000'000ULL + ts_now.tv_nsec;

        traj_gen.compute(t, pos, vel);

        for (int i = 0; i < N_JOINTS; ++i) {
            msg.positions[i] = pos[i];
            msg.velocities[i] = vel[i];
            msg.efforts[i]    = 0.0;
        }

        if (write(s, &msg, sizeof(msg)) != sizeof(msg))
            perror("client write");

        clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_ts, NULL);
    }

    return nullptr;
}