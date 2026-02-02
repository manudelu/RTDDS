#include "rt_sockets.hpp"
#include "JointStateMsg.hpp"
#include "TrajectoryGenerator.hpp"
#include "TrajectoryTypes.hpp"
#include <time.h>
#include <stdint.h>
#include <unistd.h>

constexpr size_t MAX_DOFS = SpotRobot::dofs;
constexpr double end_pos[MAX_DOFS] = { 
    0.0, 0.4, -0.5, 
    0.0, 0.5, -0.4, 
    0.0, 0.4, -0.5, 
    0.0, 0.5, -0.4 
};

void* client(void* arg) {
    SpotJointStateMsg msg;
    uint64_t seq = 0;

    TrajectoryGenerator<Quintic, MAX_DOFS> traj_gen(spot, end_pos, 5.0);

    int s = create_rt_socket(IDDP_PORT_LABEL, true, RtSocketRole::Client);

    struct timespec sleep_ts = {0, 50'000'000}; // 20Hz
    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    double pos[MAX_DOFS], vel[MAX_DOFS];

    struct timespec ts_now;
    double t;

    for(;;) {
        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        t = (ts_now.tv_sec - t0.tv_sec) + (ts_now.tv_nsec - t0.tv_nsec)/1e9;

        msg.seq = seq++;
        msg.timestamp_ns = ts_now.tv_sec * 1'000'000'000ULL + ts_now.tv_nsec;

        traj_gen.compute(t, pos, vel);

        for (int i = 0; i < MAX_DOFS; ++i) {
            msg.positions[i]  = pos[i];
            msg.velocities[i] = vel[i];
            msg.efforts[i]    = 0.0;
        }

        if (write(s, &msg, sizeof(msg)) != sizeof(msg))
            error_handler("write");

        clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_ts, NULL);
    }

    return nullptr;
}