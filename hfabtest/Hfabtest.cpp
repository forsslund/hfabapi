//
// Example for usage of the Haptikfabriken API 2023 edition
//
// There are two ways to interface with the API after calling open().
// You can either implement a loop that calls getPos() and end with setForce(),
// or you can use the built-in thread that calls a callback function of your choice.
//
#define WINDOWS
#include <iostream>
#include "../hfab_api.h"

#include <conio.h> // for _kbhit


using namespace haptikfabriken;

// A callback function that computes a force given a position (and optionally orientation) of the
// device. 
fsVec3d myHapticFunction(derived_values values) {
    // get the position
    fsVec3d pos = values.pos;

    // compute a force to the center of the workspace
    fsVec3d force = 100 * (fsVec3d(0,0,0) - pos);

    //std::cout << "pos: " << toString(pos) <<  " force: " << toString(force) << std::endl;
    std::cout << "model: " << values.raw.model << " encoders (before offset): " 
        << values.raw.enc[0] << " "
        << values.raw.enc[1] << " "
        << values.raw.enc[2] << " "
        << values.raw.enc[3] << " "
        << values.raw.enc[4] << " "
        << values.raw.enc[5] << " Error: "
        << values.raw.error_code << "\n";

    // return force to be sent to device
    return force;
}

int main()
{
    std::cout << "Hello World!\n";

    HaptikfabrikenInterface hi;
    hi.open("COM4"); // Find your port name in Windows Device Manager under "Ports".


    // Example with callback below. Please note that if your function is part of a class it
    // should be registered in this way:
    // HapticController controller;
    // hi.registerCallback(std::bind(&HapticController::myHapticFunction, &controller, std::placeholders::_1));
    hi.registerCallback(myHapticFunction);
    hi.startCallback();
    while (!_kbhit()) { // do something else, it will run in background.
    }
    hi.stopCallback();


    std::cout << "\nCallback stopped. Press enter to continue.\n";
    char c;
    std::cin >> c;


    // Alternatively make your own loop:
    while (!_kbhit()) {
        fsVec3d pos = hi.getPos();
        fsVec3d force = 100 * (fsVec3d(0,0,0) - pos);
        std::cout << "pos: " << toString(pos) <<  " force: " << toString(force) << std::endl;
        hi.setForce(force);
    }

    hi.close();
}
