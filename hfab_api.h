#ifndef HFAB_API
#define HFAB_API
#include <stdio.h>
#include "hfab_protocol.h"
#include "hfab_math.h"


#pragma warning(disable:4996)

#include <iostream>
#include <bitset>
#include <string>
#include <functional>

//#define DUMMY_DEVICE
//#define VERBOSE

//#define WINDOWS
//#define LINUX


namespace haptikfabriken {
static const char* version = "0.5 2023-04-05";

class PJRCSerialComm;

struct derived_values {
    fsVec3d pos; // in meters x,y,z (x forward towards user, y right, z up)
    fsRot orientation;
    double tA{ 0 };
    double lambda{ 0 };
    double tD{ 0 };
    double tE{ 0 };
};

class HaptikfabrikenInterface {
public:
    // Valid port names are: "COM2" or "/dev/ttyACM0"
    int open(std::string port); // Returns 0 if success
    void close();


    // If error, get error message here
    std::string getErrorCode();

    fsVec3d getPos();          // Get cartesian coords xyz using kineamtics model (actually polls device)
    fsRot getRot();                // Get orientaion of manipulandum (based on latest poll of device)
    void setForce(fsVec3d f);      // Set force using kineamtics model, e.g. in xyz
    std::bitset<5> getSwitchesState(); // Get state of up to 5 manipulandum switches (if available)

    device_to_pc_message getInMesg() const { return msg_in; }

    using Callback = std::function<fsVec3d(derived_values)>;
    void registerCallback(Callback cb) { callback = cb; }
    void startCallback();
    void stopCallback() { isRunning = false; }

    ~HaptikfabrikenInterface();

private:
    PJRCSerialComm* sc{ 0 };
    device_to_pc_message msg_in;     // updates after getPos() call (raw data)
    derived_values values;           // updates after getPos() call (derived data)
    fsVec3d lastSentForce;
    int get_minus_send_requests{ 0 };
    Callback callback;
    bool isRunning{ false };

    void callbackLoop();
};



}
#endif