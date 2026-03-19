#pragma once

// Define the type of motor in the car 
// #define HUB_MOTOR
#define KOFORD_MOTOR

#ifdef HUB_MOTOR
    constexpr int SUPPLY_VOLTAGE {48};
    constexpr int POLE_PAIRS {15};
    constexpr float ZERO_ALIGN_VALUE {5.24f};
    // Missing adjust 
    constexpr float PHASE_RESISTANCE {0.005f};
    constexpr float KV_RATING        {47.5f};
    // Safety values
    constexpr float VOLTAGE_LIMIT {48.0f};
    constexpr float CURRENT_LIMIT {8.0f};
    // Controller Iq gains
    constexpr float IQ_KP {3.0f};
    constexpr float IQ_KI {500.0f};
    constexpr float IQ_KD {0.0f};
    // Controller Id gains
    constexpr float ID_KP {0.1f};
    constexpr float ID_KI {100.0f};
    constexpr float ID_KD {0.0f};
    // Filter time constants (TF)
    constexpr float IQ_TF {0.1f};
    constexpr float ID_TF {0.1f};
#endif /* HUB_MOTOR */

#ifdef KOFORD_MOTOR
    constexpr int SUPPLY_VOLTAGE {24};
    constexpr int POLE_PAIRS {5};
    constexpr float ZERO_ALIGN_VALUE {5.24};
    // Missing adjust 
    constexpr float PHASE_RESISTANCE {0.062f};
    constexpr float KV_RATING        {100.0f};
    // Safety values
    constexpr float VOLTAGE_LIMIT {24.0f};
    constexpr float CURRENT_LIMIT {3.0f};
    // Controller Iq gains
    constexpr float IQ_KP {4.0f};
    constexpr float IQ_KI {200.0f};
    constexpr float IQ_KD {0.0f};
    // Controller Id gains
    constexpr float ID_KP {3.0f};
    constexpr float ID_KI {200.0f};
    constexpr float ID_KD {0.0f};
    // Filter time constants (TF)
    constexpr float IQ_TF {0.1f};
    constexpr float ID_TF {0.1f};
#endif /* KOFORD_MOTOR */

// Current sensing (inline) parameters 
constexpr float SHUNT_VALUE       {0.005};
constexpr int CURRENT_SENSOR_GAIN {20};