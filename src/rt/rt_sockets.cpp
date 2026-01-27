#include "rt_sockets.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

void error_handler(const char* error) {
    perror(error);
    exit(EXIT_FAILURE);
}

int create_rt_socket(const char* label, bool is_iddp, RtSocketRole role) {
    int proto = is_iddp ? IPCPROTO_IDDP : IPCPROTO_XDDP;
    int optname = is_iddp ? IDDP_LABEL : XDDP_LABEL;
    int sol = is_iddp ? SOL_IDDP : SOL_XDDP;

    // Create socket
    int sock = socket(AF_RTIPC, SOCK_DGRAM, proto);
    if (sock < 0) error_handler("socket");

    // Set label
    struct rtipc_port_label port_label;
    memset(&port_label, 0, sizeof(port_label));
    strncpy(port_label.label, label, sizeof(port_label.label) - 1);

    if (setsockopt(sock, sol, optname, &port_label, sizeof(port_label)) < 0)
        error_handler("setsockopt");

    // Bind/Connect socket
    struct sockaddr_ipc addr;
    memset(&addr, 0, sizeof(addr));
    addr.sipc_family = AF_RTIPC;
    addr.sipc_port = -1;  // -1 = dynamic assigned port from kernel

    if (role == RtSocketRole::Server) {
        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            error_handler("bind");
    } else {
        if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
            error_handler("connect");
    }

    return sock;
}

int open_device(const char* label, int flags) {
    char *device;
    if (asprintf(&device, "/proc/xenomai/registry/rtipc/xddp/%s", label) < 0) 
        error_handler("asprintf");
    int fd = open(device, flags);
    free(device);
    if (fd < 0) error_handler("open");
    return fd;
}