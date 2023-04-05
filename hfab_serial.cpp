#include "hfab_serial.h"
#include <iostream>

namespace haptikfabriken {

constexpr int buf_len = 127;

// -----------------------------------------------------------------------------
// PJRC Code below
// -----------------------------------------------------------------------------
PORTTYPE open_port_and_set_baud_or_die(const char *name, long baud);
int transmit_bytes(PORTTYPE port, const char *data, int len);
int receive_bytes(PORTTYPE port, char *buf, int len);
int receive_bytes_until_bracket(PORTTYPE port, char *buf);
void close_port(PORTTYPE port);
void delay(double sec);

void delay(double sec)
{
#if defined(MACOSX) || defined(LINUX)
    usleep(sec * 1000000);
#elif defined(WINDOWS)
    Sleep(DWORD(sec * 1000));
#endif
}

// wait for the Arduino board to boot up, since opening
// the board raises DTR, which resets the processor.
// as soon as it properly responds, we know it's running
void wait_online(PORTTYPE port)
{
    char buf[buf_len+1];
    memset(buf,0,buf_len+1);
    int r;

    printf("waiting for board to be ready:\n");
    while (1) {
        delay(0.1);
        printf(".");
        fflush(stdout);
        sprintf(buf,"[0,0,0]");
        transmit_bytes(port, buf, 16); // always send at least 16 bytes (at least in windows)

        r = receive_bytes(port, buf, buf_len);
        printf("%d\n",r);
        if (r > 8) break; // success, device online
    }
    printf("ok\n");
}


PORTTYPE open_port_and_set_baud_or_die(const char *name, long baud)
{
    PORTTYPE fd;
#if defined(MACOSX)
    struct termios tinfo;
    fd = open(name, O_RDWR | O_NONBLOCK);
    if (fd < 0)printf("unable to open port %s\n", name);
    if (tcgetattr(fd, &tinfo) < 0)printf("unable to get serial parms\n");
    if (cfsetspeed(&tinfo, baud) < 0)printf("error in cfsetspeed\n");
    tinfo.c_cflag |= CLOCAL;
    if (tcsetattr(fd, TCSANOW, &tinfo) < 0)printf("unable to set baud rate\n");
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
#elif defined(LINUX)
    struct termios tinfo;
    struct serial_struct kernel_serial_settings;
    int r;
    fd = open(name, O_RDWR);
    if (fd < 0)printf("unable to open port %s\n", name);
    if (tcgetattr(fd, &tinfo) < 0)printf("unable to get serial parms\n");
    cfmakeraw(&tinfo);
    if (cfsetspeed(&tinfo, baud) < 0)printf("error in cfsetspeed\n");
    if (tcsetattr(fd, TCSANOW, &tinfo) < 0)printf("unable to set baud rate\n");
    r = ioctl(fd, TIOCGSERIAL, &kernel_serial_settings);
    if (r >= 0) {
        kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
        r = ioctl(fd, TIOCSSERIAL, &kernel_serial_settings);
        if (r >= 0) printf("set linux low latency mode\n");
    }
#elif defined(WINDOWS)
    COMMCONFIG cfg;
    COMMTIMEOUTS timeout;
    DWORD n;
    char portname[256];
    int num;
    if (sscanf_s(name, "COM%d", &num) == 1) {
        sprintf(portname, "\\\\.\\COM%d", num); // Microsoft KB115831
    } else {
        strncpy_s(portname, name, sizeof(portname)-1);
        //portname[n-1] = 0;
    }
    fd = CreateFileA(portname, GENERIC_READ | GENERIC_WRITE,
        0, 0, OPEN_EXISTING, 0, NULL);
    GetCommConfig(fd, &cfg, &n);
    cfg.dcb.BaudRate = baud;
    //cfg.dcb.BaudRate = 115200;
    cfg.dcb.fBinary = TRUE;
    cfg.dcb.fParity = FALSE;
    cfg.dcb.fOutxCtsFlow = FALSE;
    cfg.dcb.fOutxDsrFlow = FALSE;
    cfg.dcb.fOutX = FALSE;
    cfg.dcb.fInX = FALSE;
    cfg.dcb.fErrorChar = FALSE;
    cfg.dcb.fNull = FALSE;
    cfg.dcb.fRtsControl = RTS_CONTROL_ENABLE;
    cfg.dcb.fAbortOnError = FALSE;
    cfg.dcb.ByteSize = 8;
    cfg.dcb.Parity = NOPARITY;
    cfg.dcb.StopBits = ONESTOPBIT;
    cfg.dcb.fDtrControl = DTR_CONTROL_ENABLE;
    SetCommConfig(fd, &cfg, n);
    GetCommTimeouts(fd, &timeout);
    timeout.ReadIntervalTimeout = 0;
    timeout.ReadTotalTimeoutMultiplier = 0;
    timeout.ReadTotalTimeoutConstant = 1000;
    timeout.WriteTotalTimeoutConstant = 0;
    timeout.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(fd, &timeout);
#endif
    return fd;

}


int successes_before_timeout{ 0 };
int receive_bytes_until_bracket(PORTTYPE port, char *buf)
{
    int count=0;
    int len=1; // easiset, add more later to speedup?
#if defined(MACOSX) || defined(LINUX)
    int waiting=0;

    // Set non-blocking
    fcntl(port, F_SETFL, fcntl(port, F_GETFL) | O_NONBLOCK);
    while (true) {
        int n = read(port, buf + count, 64); // may return -1
        if (n > 0){
            count += n;
            bool found = false;
            for (unsigned int c = 0; c < count; ++c)
                if (buf[c] == ']') found = true;
            if (found) {
                // flush
                //while(n>0)
                //    ReadFile(port, buf++, 1, &n, NULL);
                successes_before_timeout++;

                break;
            }
        }
        else {
            if (waiting) {
#ifdef VERBOSE
                std::cout << "Linux timeout read... "<< successes_before_timeout << " sucesses before\n";
#endif
                successes_before_timeout = 0;
                break;  // 1 sec timeout
            }

            // Set blocking
            fd_set fds;
            struct timeval t;
            FD_ZERO(&fds);
            FD_SET(port, &fds);
            t.tv_sec = 0;
            t.tv_usec = 100;
            int sr = select(port+1, &fds, NULL, NULL, &t);
            fcntl(port, F_SETFL, fcntl(port, F_GETFL) & ~O_NONBLOCK);

            waiting = 1;
        }
    }
#elif defined(WINDOWS)
    COMMTIMEOUTS timeout;
    DWORD n;
    BOOL r;
    int waiting=0;

    GetCommTimeouts(port, &timeout);
    timeout.ReadIntervalTimeout = MAXDWORD; // non-blocking
    timeout.ReadTotalTimeoutMultiplier = 0;
    timeout.ReadTotalTimeoutConstant = 0;
    SetCommTimeouts(port, &timeout);
    buf[0]=0;
    while (true) {
        //r = ReadFile(port, buf + count, len - count, &n, NULL);
        r = ReadFile(port, buf + count, 64, &n, NULL);
        bool found = false;
        for (unsigned int c = 0; c < count + n; ++c)
            if (buf[c] == ']') found = true;
        if (found) {
            // flush
            //while(n>0)
            //    ReadFile(port, buf++, 1, &n, NULL);
            count += n;
            successes_before_timeout++;

            break;
        }
        if (n > 0) count += n;
        else {
            if (waiting) {
#ifdef VERBOSE
                std::cout << "Windows timeout read... "<< successes_before_timeout << " sucesses before\n";
#endif
                successes_before_timeout = 0;
                break;  // 1 sec timeout
            }
            timeout.ReadIntervalTimeout = MAXDWORD;
            timeout.ReadTotalTimeoutMultiplier = MAXDWORD;
            timeout.ReadTotalTimeoutConstant = 1000;
            SetCommTimeouts(port, &timeout);
            waiting = 1;
        }
    }
#endif
    return count;
}






int receive_bytes(PORTTYPE port, char *buf, int len)
{
    int count=0;
#if defined(MACOSX) || defined(LINUX)
    int r;
    int retry=0;
    //char buf[512];

    //if (len > sizeof(buf) || len < 1) return -1;
    // non-blocking read mode
    fcntl(port, F_SETFL, fcntl(port, F_GETFL) | O_NONBLOCK);
    while (count < len) {
        r = read(port, buf + count, len - count);
//        total_reads++;
//        printf("read, r = %d, len = %d, count = %d, (retry %d) (total reads %d)\n", r, len, count, retry, total_reads);
        if (r < 0 && errno != EAGAIN && errno != EINTR) {
            //std::cout << "Error receiving bytes (errno " << errno << ")\n";
            return -1;
        }
        else if (r > 0) count += r;
        else if(retry>500){
            //std::cout << "no data available right now, must wait\n";
//            std::cout << "no data available right now, must wait. Errno:  " << errno << ", r: " << r << ", retry: " << retry << ", Total reads: " << total_reads << "\n";
            // no data available right now, must wait
            
            fd_set fds;
            struct timeval t;
            FD_ZERO(&fds);
            FD_SET(port, &fds);
            t.tv_sec = 0;
            t.tv_usec = 100;
            r = select(port+1, &fds, NULL, NULL, &t);
            //printf("select, r = %d\n", r);
            if (r < 0) {
               // printf("Select error: %d\n", r);
                return -1;
            }
            if (r == 0){
//                printf("Timeout! (count: %d)\n", count);
                return count; // timeout

            }
        }
        retry++;
        if (retry > 2000) return -100; // no input
    }
    fcntl(port, F_SETFL, fcntl(port, F_GETFL) & ~O_NONBLOCK);
#elif defined(WINDOWS)
    COMMTIMEOUTS timeout;
    DWORD n;
    BOOL r;
    int waiting=0;

    GetCommTimeouts(port, &timeout);
    timeout.ReadIntervalTimeout = MAXDWORD; // non-blocking
    timeout.ReadTotalTimeoutMultiplier = 0;
    timeout.ReadTotalTimeoutConstant = 0;
    SetCommTimeouts(port, &timeout);
    while (count < len) {
        r = ReadFile(port, buf + count, len - count, &n, NULL);
        if (n > 0) count += n;
        else {
            if (waiting) break;  // 1 sec timeout
            timeout.ReadIntervalTimeout = MAXDWORD;
            timeout.ReadTotalTimeoutMultiplier = MAXDWORD;
            timeout.ReadTotalTimeoutConstant = 1000;
            SetCommTimeouts(port, &timeout);
            waiting = 1;
        }
    }
#endif
    return count;
}


int transmit_bytes(PORTTYPE port, const char *data, int len)
{
#if defined(MACOSX) || defined(LINUX)
    return write(port, data, len);
#elif defined(WINDOWS)
    DWORD n;
    BOOL r;
    r = WriteFile(port, data, len, &n, NULL);
    if (!r) return 0;
    return n;
#endif
}


void close_port(PORTTYPE port)
{
#if defined(MACOSX) || defined(LINUX)
    close(port);
#elif defined(WINDOWS)
    CloseHandle(port);
#endif
}





void PJRCSerialComm::open(std::string port) {
    fd = open_port_and_set_baud_or_die(port.c_str(), BAUD);
    wait_online(fd);
    char buf[1];
            int aa=0;
            int bb=0;
            while(bb=receive_bytes(fd, buf, 1)>0){aa+=bb;}
            printf("Flushed %d bytes\n",aa);
}

void PJRCSerialComm::close(){
    close_port(fd);
}

void printBuf(char* buf, std::string str, bool header=true, int max_length=0){
    if(header)
        std::cout << str << " [0123456789012345678901234567890123456789012345678901234567890123]\n";
    //std::cout << str << " \"";
    int i;
    for(i=0;i<(max_length?max_length:(buf_len+1));++i){
        if(buf[i]==0) { std::cout << "#"; continue; };
        if(buf[i]=='\n') { std::cout << ";"; continue; }
        if(buf[i]=='\r') { std::cout << "R"; continue; }
        if(buf[i]==' ')  { std::cout << "_"; continue; }
        std::cout << buf[i];
    }
    //std::cout << "\" Size (including null character): " << i <<"\n";
}

void PJRCSerialComm::send(const pc_to_device_message &msg)
{
    char buf[buf_len+1];
    memset(buf, '\0', buf_len+1);
    int len = msg.toChars(buf);
#ifdef VERBOSE
    std::cout << "send " << len << " bytes: " << std::string(buf) << std::endl;
#endif
    transmit_bytes(fd, buf, len<16?16:len); // Yes, this speeds up transfer for unknown reason
}


int PJRCSerialComm::receive(device_to_pc_message &msg)
{
    constexpr int len=buf_len;
    char buf[len+1];
    memset(buf, '\0', len+1);
    int n = receive_bytes_until_bracket(fd,buf);

#ifdef VERBOSE
    std::cout << "n: " << n << "-->" << std::string(buf) << std::endl;
    if (n == 0) delay(10);
#endif
    int parse_error = msg.fromChars(buf);
    if(parse_error){
        std::cout << "Parse error! n: " << n << "-->" << std::string(buf) << std::endl;
#ifdef VERBOSE
        if (n == 0) delay(10);
#endif
    }

#ifdef VERBOSE
    if(msg.error_code) // PRINT ALL FOR LOGGING PURPOSES 
        printBuf(buf, "",false,n);
#endif
        return 0; // success
}



}


