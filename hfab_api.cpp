#include "hfab_api.h"
#include "hfab_math.h"
#include "hfab_serial.h"
#include <thread>

#if defined(WINDOWS)
#include <windows.h>
#else
#include <dirent.h>
#include <unistd.h>
#include <climits>
#endif

namespace haptikfabriken {

std::string HaptikfabrikenInterface::serialport_name;
std::string HaptikfabrikenInterface::serialport_names[10];

unsigned int HaptikfabrikenInterface::findUSBSerialDevices() {
    serialport_name.clear();
    for (int i = 0; i < 10; ++i) serialport_names[i].clear();

#if defined(WINDOWS)
    // Probe COM3..COM20; a device is present if the port can be opened.
    unsigned int found = 0;
    for (int i = 3; i <= 20 && found < 10; ++i) {
        std::string port = "\\\\.\\COM" + std::to_string(i);
        HANDLE h = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                               NULL, OPEN_EXISTING, 0, NULL);
        if (h != INVALID_HANDLE_VALUE) {
            CloseHandle(h);
            if (found == 0) serialport_name = "COM" + std::to_string(i);
            serialport_names[found++] = "COM" + std::to_string(i);
        }
    }
    return found;
#else
    // Linux/Mac: the by-id directory only exists while a USB serial device is
    // connected, so opendir fails fast (returns 0) when nothing is plugged in.
    DIR* dir = opendir("/dev/serial/by-id");
    if (dir == nullptr) return 0;

    unsigned int found = 0;
    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr && found < 10) {
        std::string name = ent->d_name;
        if (name.find("Teensy") == std::string::npos) continue;

        char buf[PATH_MAX];
        std::string link = std::string("/dev/serial/by-id/") + name;
        ssize_t n = readlink(link.c_str(), buf, sizeof(buf) - 1);
        if (n <= 0) continue;
        buf[n] = '\0';

        std::string target(buf);                          // e.g. "../../ttyACM0"
        size_t slash = target.find_last_of('/');
        std::string node = "/dev/" + (slash == std::string::npos
                                          ? target : target.substr(slash + 1));
        if (found == 0) serialport_name = node;
        serialport_names[found++] = node;
    }
    closedir(dir);
    return found;
#endif
}

// -----------------------------------------------------------------------------
// Haptikfabriken code
// -----------------------------------------------------------------------------

std::string HaptikfabrikenInterface::getErrorCode() {
    return std::string("error");
}

HaptikfabrikenInterface::~HaptikfabrikenInterface() { if (sc) delete sc; }

int HaptikfabrikenInterface::open(std::string port){
#ifdef DUMMY_DEVICE
    std::cout << "Dummy device: open("<<port<<")\n";    
#else

    if (!sc) sc = new PJRCSerialComm();
    sc->open(port);
    sc->sendWakeupMessage();
#endif

    return 0;
}

void HaptikfabrikenInterface::close(){
    #ifdef DUMMY_DEVICE
    std::cout << "Dummy device: close()\n";
    return;
#endif
    if (!sc) return;
    sc->close();
    delete sc;
    sc = 0;
}

// Now set within kinematics object
//Kinematics kinematics(Kinematics::configuration::vintage());
constexpr int enc_home_vintage[] = { 0,7700,0,0,0,0 };
Kinematics kinematics(Kinematics::configuration::polhem_v3()); // polhem_v3() in haptikfabriken.h!
constexpr int enc_home_polhem[] = { 8312, -10366, 19764, 0, 30, 0 };
constexpr int enc_home_woody[] = { 0, 0, -6000, 0, 0, 0 };

derived_values compute_values(const device_to_pc_message& m) {
    derived_values out;
    out.raw = m;

    // receive encoders, put in counter[0]-[5]
    int counter[] = { m.enc[0],m.enc[1],m.enc[2],m.enc[3],m.enc[4],m.enc[5] };

    const int base[] = { counter[0], counter[1], counter[2] };
    
    fsVec3d p = kinematics.computePosition(base);

    const int rot[] = { counter[3],counter[4],counter[5] };
    volatile double latest_angles[] = { 0,0,0,0 }; // tA,lambda,tD,tE
    kinematics.computeRotation(base, rot, latest_angles);

    out.tA = latest_angles[0];
    out.lambda = latest_angles[1];
    out.tD = latest_angles[2];
    out.tE = latest_angles[3];
    out.pos.m_x = p.m_x;
    out.pos.m_y = p.m_y;
    out.pos.m_z = p.m_z;
    
    return out;
}


fsVec3d HaptikfabrikenInterface::getPos(){
    #ifdef VERBOSE
        std::cout << "HaptikfabrikenInterface::getPos()... ";
    #endif

    // Check if device is waiting for a force command (ie. we have called getPos() twice)
    if (get_minus_send_requests>0) {
        std::cout << "Note: get_minus_send_requests: " << get_minus_send_requests << "\n";
    #ifdef VERBOSE
        std::cout << "Note: get_minus_send_requests: " << get_minus_send_requests << "\n";
    #endif
        setForce(lastSentForce);
        get_minus_send_requests = 0;
    }
    get_minus_send_requests++;
    #ifdef DUMMY_DEVICE
        return fsVec3d(0.01, 0.02, 0.03);
    #endif
    device_to_pc_message new_msg;

    if(sc->receive(new_msg)){ // Error reading, use
        std::cout << "in error reading\n";
    } else {

        // have we changed kinematics?
        if(kinematics.model != new_msg.model){
            if(new_msg.model==1){
                kinematics = Kinematics(Kinematics::configuration::polhem_v3());
                for(int i=0;i<6;++i) kinematics.enc_home[i] = enc_home_polhem[i];
            }
            else if(new_msg.model==2){
                kinematics = Kinematics(Kinematics::configuration::vintage());
                for(int i=0;i<6;++i) kinematics.enc_home[i] = enc_home_vintage[i];
            }
            else if(new_msg.model==3){
                kinematics = Kinematics(Kinematics::configuration::woodenhaptics_v2015());
                for(int i=0;i<6;++i) kinematics.enc_home[i] = enc_home_woody[i];
            } else
                std::cout << "Unsupported kinematics model " << new_msg.model << "\n";
            kinematics.model = new_msg.model;
        }

        // Add the encoder home position offset
        for(int i=0;i<6;++i)
            new_msg.enc[i] += kinematics.enc_home[i];

        msg_in = new_msg;

        values = haptikfabriken::compute_values(new_msg);
    }
    #ifdef VERBOSE
        std::cout << "returns: " << msg.x_mm << " " << msg.y_mm << " " << msg.z_mm << "\n";
    #endif
    return values.pos;
}

std::bitset<5> HaptikfabrikenInterface::getSwitchesState(){
    bool btn = false;

    // TODO: implement vintage button

    std::bitset<5> switches;
    switches[0]=btn;
    return switches;
}

fsRot HaptikfabrikenInterface::getRot(){
    double tF = 0;

    int tF_count = msg_in.enc[5];
    if(kinematics.model==model_vintage_raw)
        tF = 2 * 3.1415926535897 * tF_count / 2000;
    else
        tF = 2 * 3.1415926535897 * tF_count / 1024;


    fsRot rA, rC, rD, rE, rF;
    rA.rot_z(values.tA);
    rC.rot_y(-values.lambda);
    if(kinematics.model==model_vintage_raw)
        rD.rot_z(values.tD);
    else
        rD.rot_x(values.tD);
    rE.rot_y(values.tE);
    rF.rot_x(tF);

    return rA * rC * rD * rE * rF;
}

void HaptikfabrikenInterface::setForce(fsVec3d f){
    #ifdef VERBOSE
        std::cout << "HaptikfabrikenInterface::setForce(" << toString(f) << ")\n";
    #endif

    get_minus_send_requests--;
    if (get_minus_send_requests < 0) {
#ifdef VERBOSE
        std::cout << "Note, in set force: get_minus_send_requests: " << get_minus_send_requests << "\n";
#endif
        getPos();
    }
    lastSentForce = f;

#ifdef DUMMY_DEVICE
    return;
#endif
    pc_to_device_message out;
    int base[] = {msg_in.enc[0],msg_in.enc[1],msg_in.enc[2]};
    fsVec3d a = kinematics.computeMotorAmps(f, base);
    out.ma[0]=int(a.m_x*1000);
    out.ma[1]=int(a.m_y*1000);
    out.ma[2]=int(a.m_z*1000);
    sc->send(out);
}

void HaptikfabrikenInterface::startCallback() {
    if (!isRunning) {
        isRunning = true;
        std::thread sensorThread(&HaptikfabrikenInterface::callbackLoop, this);
        sensorThread.detach();
    }
}

void HaptikfabrikenInterface::callbackLoop() {
    while (isRunning) {
        getPos(); // update values
        values.orientation = getRot();
        fsVec3d force;
        if (callback) {
            force = callback(values);
        }
        setForce(force);
    }
}

}