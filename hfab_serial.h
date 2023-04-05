#ifndef HFAB_SERIAL
#define HFAB_SERIAL
#define WINDOWS
#pragma warning(disable:4996)
#include <iostream>

// -----------------------------------------------------------------------------
// Includes (from PJRC)
// -----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#if defined(MACOSX) || defined(LINUX)
#include <unistd.h> // usleep
#include <termios.h>
#include <sys/select.h>
#include <math.h>
#define PORTTYPE int
#define BAUD B115200
#if defined(LINUX)
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif
#elif defined(WINDOWS)
#ifndef _WINSOCKAPI_
#define DID_DEFINE_WINSOCKAPI
#define _WINSOCKAPI_
#endif
#include <windows.h>
#define PORTTYPE HANDLE
#define BAUD 115200
#else
#error "You must define the operating system\n"
#endif

/*
#if defined(MACOSX) || defined(LINUX)
#define PORTTYPE int
#elif defined(WINDOWS)
#define PORTTYPE HANDLE
#else
#error "You must define the operating system\n"
#endif
*/

#include "hfab_protocol.h"

namespace haptikfabriken {



class PJRCSerialComm {
public:
    void sendWakeupMessage(){pc_to_device_message m; send(m);}
    void open(std::string portname);
    void close();
    void send(const pc_to_device_message& msg);
    int receive(device_to_pc_message& msg);
private:
    PORTTYPE fd;
    short last_info{0};
};

}


#endif