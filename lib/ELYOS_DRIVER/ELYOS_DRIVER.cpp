#include "ELYOS_DRIVER.h"

static ELYOS_DRIVER* elyos_instance = nullptr;

////////// HALL CALLLBACKS //////////
void hall_A_Handle(){
    if(elyos_instance && elyos_instance->hall_sensor)
        elyos_instance->hall_sensor->handleA();
}

void hall_B_Handle(){
    if(elyos_instance && elyos_instance->hall_sensor)
        elyos_instance->hall_sensor->handleB();
}

void hall_C_Handle(){
    if(elyos_instance && elyos_instance->hall_sensor)
        elyos_instance->hall_sensor->handleC();
}

///////// COMMANDER CALLBACK /////////
void ELYOS_DRIVER::do_Motor(char *cmd){
    this->commander->motor(this->motor, cmd);  // Check if passed correcly
}

void onIdPWrapper(char* cmd){
    if(elyos_instance) elyos_instance->cmd_set_Id_P(cmd);
}

void onIdIWrapper(char* cmd){
    if(elyos_instance) elyos_instance->cmd_set_Id_I(cmd);
}

void onIqPWrapper(char* cmd){
    if(elyos_instance) elyos_instance->cmd_set_Iq_P(cmd);
}

void onIqIWrapper(char* cmd){
    if(elyos_instance) elyos_instance->cmd_set_Iq_I(cmd);
}

//////////////////////////////////////


int ELYOS_DRIVER::driver_Init(){
    elyos_instance = this;

    // Initialize hardware 
    motor = new BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING, 0.0012f);    // OPTIONAL: Winding resistance, KV, Inductances 
    driver = new BLDCDriver6PWM(A_PHASE_HIGH_PIN, A_PHASE_LOW_PIN,
                                B_PHASE_HIGH_PIN, B_PHASE_LOW_PIN,
                                C_PHASE_HIGH_PIN, C_PHASE_LOW_PIN);
    
    // Sensors 
    hall_sensor = new HallSensor(HALL_A_PIN, HALL_C_PIN, HALL_B_PIN, POLE_PAIRS);
    current_sense = new InlineCurrentSense(SHUNT_VALUE, CURRENT_SENSOR_GAIN, CURRENT_SENSE_A_PIN,
                                           CURRENT_SENSE_B_PIN, CURRENT_SENSE_C_PIN);


    // Commander to tune controller quickly 
    commander = new Commander(Serial);
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    commander->add('a', onIdPWrapper, "Id P");
    commander->add('b', onIdIWrapper, "Id I");
    commander->add('c', onIqPWrapper, "Iq P");
    commander->add('d', onIqIWrapper, "Iq I");

    // Throttle 
    pinMode(THROTTLE_PIN, INPUT);
    analogReadResolution(12);

    // Hall sensor 
    hall_sensor->init();
    hall_sensor->enableInterrupts(hall_A_Handle, hall_B_Handle, hall_C_Handle);
    motor->linkSensor(hall_sensor);

    // Driver init
    driver->voltage_power_supply = SUPPLY_VOLTAGE;
    driver->pwm_frequency = 20000;

    // Check if init is successful
    if(!driver->init()){
        return ELYOS_DRIVER_ERROR;
    }
    motor->linkDriver(driver);

    // Current sense init 
    current_sense->linkDriver(driver);
    if(!current_sense->init()){
        return ELYOS_DRIVER_ERROR;
    }
    // motor->linkCurrentSense(this->current_sense);

    // Safety 
    motor->voltage_limit = VOLTAGE_LIMIT;
    motor->current_limit = CURRENT_LIMIT;

    // Configure controller gains 
    control_Init();

    if(!motor->init()){
        return ELYOS_DRIVER_ERROR;
    }

    // Monitor data
    this->motor->useMonitoring(Serial);
    this->motor->monitor_downsample = 100;
    this->motor->monitor_variables = _MON_TARGET |_MON_CURR_Q | _MON_CURR_D;

    ///// TUNE THIS
    // motor->zero_electric_angle = ZERO_ALIGN_VALUE;
    motor->sensor_direction = Direction::CW;

    // Init Field Oriented Controller
    if(!motor->initFOC()){
        return ELYOS_DRIVER_ERROR;
    }

    // Safe startup
    motor->target = 0.0f;

    return ELYOS_DRIVER_OK;
}


int ELYOS_DRIVER::control_Init(){
    // this->motor->torque_controller = TorqueControlType::foc_current;
    // this->motor->controller = MotionControlType::torque;

    this->motor->torque_controller = TorqueControlType::voltage;
    this->motor->controller = MotionControlType::velocity_openloop;

    // Establish PID gains direct component 
    this->motor->PID_current_d.P = ID_KP;
    this->motor->PID_current_d.I = ID_KI;
    // this->motor->PID_current_d.D = ID_KD;

    // Establish PID gains quadrature component 
    this->motor->PID_current_q.P = IQ_KP;
    this->motor->PID_current_q.I = IQ_KI;
    // this->motor->PID_current_q.D = IQ_KD;

    // Low pass filter 
    this->motor->LPF_current_q.Tf = 0.001f;   // Small Tf = fast response, large Tf = smooth, laggy
    this->motor->LPF_current_d.Tf = 0.001f;

    return ELYOS_DRIVER_OK;
}


void ELYOS_DRIVER::runFOC(){
    this->motor->loopFOC();

    int raw = analogRead(THROTTLE_PIN);
    float current_cmd = raw * (CURRENT_LIMIT / 4095.0f);

    current_cmd = throttle_lpf(current_cmd);

    // motor->target = current_cmd; 
    motor->target += 0.002f;   // slow electrical angle ramp

    this->motor->move();
    this->motor->monitor();
    commander->run();
}


/////// SET PID GAINS Id
void ELYOS_DRIVER::cmd_set_Id_P(char* cmd){
    float val = atof(cmd);
    motor->PID_current_d.P = val;
    Serial.print("Id P set to ");
    Serial.println(val);
}

void ELYOS_DRIVER::cmd_set_Id_I(char* cmd){
    float val = atof(cmd);
    motor->PID_current_d.I = val;
    Serial.print("Id I set to ");
    Serial.println(val);
}

void ELYOS_DRIVER::cmd_set_Id_D(char* cmd){
    float val = atof(cmd);
    motor->PID_current_d.D = val;
    Serial.print("Id D set to ");
    Serial.println(val);
}

/////// SET PID GAINS Iq
void ELYOS_DRIVER::cmd_set_Iq_P(char* cmd){
    float val = atof(cmd);
    motor->PID_current_q.P = val;
    Serial.print("Iq P set to ");
    Serial.println(val);
}

void ELYOS_DRIVER::cmd_set_Iq_I(char* cmd){
    float val = atof(cmd);
    motor->PID_current_q.I = val;
    Serial.print("Iq I set to ");
    Serial.println(val);
}

void ELYOS_DRIVER::cmd_set_Iq_D(char* cmd){
    float val = atof(cmd);
    motor->PID_current_q.D = val;
    Serial.print("Iq D set to ");
    Serial.println(val);
}