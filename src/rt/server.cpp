#include "rt_sockets.hpp"
#include "JointStateMsg.hpp"
#include <stdio.h>

void* server(void* arg) {
    bool is_iddp = true;
    int iddp_s = create_rt_socket(IDDP_PORT_LABEL, is_iddp, RtSocketRole::Server);
    int xddp_s = create_rt_socket(XDDP_PORT_LABEL, !is_iddp, RtSocketRole::Server);

    SpotJointStateMsg msg;
    for(;;) {
        if (recv(iddp_s, &msg, sizeof(msg), 0) <= 0)
            error_handler("server recv");

        if (sendto(xddp_s, &msg, sizeof(msg), 0, NULL, 0) != sizeof(msg))
            perror("xddp sendto");
    }

    return nullptr;
}