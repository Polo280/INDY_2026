#pragma once 
#include "Pinout.h"
#include "FOC_Parameters.h"
#include <SimpleFOC.h>
#include "Telemetry_Manager.h"
#include "BLDC_Logger.h"

#define ELYOS_DRIVER_OK      0
#define ELYOS_DRIVER_ERROR  -1

typedef struct {
    int code;
    String message;
} ELYOS_DRIVER_STATUS;

class ELYOS_DRIVER {
    protected:
        // Simple FOC handlers 
        BLDCMotor *motor;
        BLDCDriver6PWM *driver;
        InlineCurrentSense *current_sense;
        // Change motor parameters during tunning easily 
        Commander *commander;
        // Low pass filter for throttle
        LowPassFilter throttle_lpf = LowPassFilter(0.02f);

    public:
        // For ISR
        HallSensor *hall_sensor;

        int driver_Init();
        int control_Init();
        void runFOC();

        // Callbacks
        void do_Motor(char *cmd);
        // Id PID
        void cmd_set_Id_P(char* cmd);
        void cmd_set_Id_I(char* cmd);
        void cmd_set_Id_D(char* cmd);
        // Iq PID
        void cmd_set_Iq_P(char* cmd);
        void cmd_set_Iq_I(char* cmd);
        void cmd_set_Iq_D(char* cmd);
};