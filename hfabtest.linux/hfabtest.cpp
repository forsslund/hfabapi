//
// Example for usage of the Haptikfabriken API 2023 edition (Linux version)
//
// This example uses the local polling loop (no callback).
// It calls getPos() and setForce() in a loop until a key is pressed.
//
#ifndef LINUX
#define LINUX
#endif
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include "../hfab_api.h"

using namespace haptikfabriken;

// Non-blocking keyboard hit check (Linux replacement for _kbhit)
bool kbhit() {
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

int main()
{
    std::cout << "Hello World!\n";

    // Set terminal to raw mode so we can detect keypresses without enter
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    HaptikfabrikenInterface hi;
    // Find your port name with: ls /dev/ttyACM*
    hi.open("/dev/ttyACM0");

    // Polling loop: read position, compute spring force, send to device
    std::cout << "Running haptic loop. Press any key to stop.\n";
    while (!kbhit()) {
        fsVec3d pos = hi.getPos();
        auto raw = hi.getInMesg();
        fsVec3d force = 100 * (fsVec3d(0,0,0) - pos);
        printf("pos: (%7.3f,%7.3f,%7.3f)  enc: [%6d,%6d,%6d,%4d,%4d,%4d] model=%d err=%d  force: %s\r",
               pos.m_x, pos.m_y, pos.m_z,
               raw.enc[0], raw.enc[1], raw.enc[2], raw.enc[3], raw.enc[4], raw.enc[5],
               raw.model, raw.error_code, toString(force).c_str());
        fflush(stdout);
        hi.setForce(force);
    }

    hi.close();

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    std::cout << "\nDone.\n";
}
