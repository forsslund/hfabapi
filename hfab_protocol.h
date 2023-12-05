#ifndef HFAB_PROTOCOL
#define HFAB_PROTOCOL
#pragma warning(disable:4996)
#include <stdio.h>
namespace haptikfabriken {
  // -----------------------------------------------------------------
  // NEW COMPUTER MESSAGE INTERFACE
  // -----------------------------------------------------------------
  constexpr int model_polhem_2022_raw = 1;
  constexpr int model_vintage_raw = 2;
  struct device_to_pc_message {
    int model{1};
    int enc[6] = {0,0,0,0,0,0};
    int error_code{0};

    // Returns number of characters, also writes a trailing \0
    int toChars(char *c) const {
      return sprintf(c, "[%d,%d,%d,%d,%d,%d,%d,%d]\n",
                     model, enc[0], enc[1], enc[2], enc[3], enc[4], enc[5],  error_code);
    }

    // Returns 0 if success, 1 if fail
    int fromChars(const char *c){
      return 8 != sscanf(c, "[%d,%d,%d,%d,%d,%d,%d,%d]",
                         &model, &enc[0], &enc[1], &enc[2], &enc[3], &enc[4], &enc[5], &error_code);
    }
  };

  //int calls{ 0 };
  struct pc_to_device_message {
    int ma[3] = {0,0,0};          // milliamps per motor

    // Returns number of characters, also writes a trailing \0
    int toChars(char *c) const {
      return sprintf(c, "[%d,%d,%d]\n", ma[0], ma[1], ma[2]); // calls++ is an option here
    }

    // Returns 1 if success, 0 if fail
    int fromChars(const char *c){
      return 3 == sscanf(c, "[%d,%d,%d]", &ma[0], &ma[1], &ma[2]);
    }
  };
  // -----------------------------------------------------------------

  



struct position_hid_to_pc_message {
    float x_mm{0};
    float y_mm{0};
    float z_mm{0};
    short a_ma{0};
    short b_ma{0};
    short c_ma{0};
    short info{0};
    float tA{0};
    float lambda{0};
    float tD{0};
    float tE{0};
  };

  struct pc_to_hid_message {  // 7*2 = 14 bytes + 1 inital byte always 0
    unsigned char reportid = 0;
    short current_motor_a_mA{0};
    short current_motor_b_mA{0};
    short current_motor_c_mA{0};
    short command{2}; // e.g. reset encoders
    short command_attr0{0};
    short command_attr1{0};
    short command_attr2{0};
  };
  // -----------------------------------------------------------------

}
#endif