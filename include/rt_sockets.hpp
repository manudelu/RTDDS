#pragma once
#include <rtdm/ipc.h>

#define IDDP_PORT_LABEL "iddp-joint-state"
#define XDDP_PORT_LABEL "xddp-joint-state"

enum class RtSocketRole { Server, Client };

void error_handler(const char* error);
int create_rt_socket(const char* label, bool is_iddp, RtSocketRole role);
int open_device(const char* label, int flags);