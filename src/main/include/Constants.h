/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

namespace constant
{
    //Motors
    static constexpr int TITAN_ID   = 42;
    static constexpr int LEFT_MOTOR = 3;    
    static constexpr int BACK_MOTOR = 1;    //BACK
    static constexpr int RIGHT_MOTOR = 0;
    static constexpr int ELEVATOR_MOTOR = 2;    //ELEVATOR

    //Encoder
    static constexpr double WHEEL_RADIUS    = 5.1; // Wheels Radius [cm]
    static constexpr double FRAME_RADIUS    = 22;  // Frame Radius [cm]
    static constexpr double PULSE_PER_REV   = 1464;
    static constexpr double GEAR_RATIO      = 1/1;
    static constexpr double ENCODER_PULSE_RATIO = PULSE_PER_REV * GEAR_RATIO;
    static constexpr double DIST_PER_TICK   =   (M_PI * 2 * WHEEL_RADIUS) / ENCODER_PULSE_RATIO;

    //Inputs
    static constexpr int START_BUTTON   = 9;
    static constexpr int STOP_BUTTON    = 8;

    //Outputs
    static constexpr int RUNNING_LED    = 12;
    static constexpr int STOPPED_LED    = 13;

    // Sharp
    static constexpr int RIGHT_SHARP    = 2;
    static constexpr int LEFT_SHARP     = 3;

    // Ultrasonic
    static constexpr int RIGHT_TRIG    = 15;
    static constexpr int RIGHT_ECHO    = 11;
    static constexpr int LEFT_TRIG     = 0;
    static constexpr int LEFT_ECHO     = 1;

    // Elevator limits
    static constexpr int ELEVATOR_LIMIT_HIGH_HEIGHT   = 35;
    static constexpr int ELEVATOR_LIMIT_LOW_HEIGHT    = 0;
    static constexpr double ELEVATOR_MAX_SPEED        = 70.0; 
    static constexpr float ELEVATOR_POSITION_TOLERANCE = 0.5; // [cm]
    static constexpr float ELEVATOR_MOVE_SPEED         = 1; // [cm]/button press in teleop mode
    static constexpr float ELEVATOR_FAST_MOVE_FACTOR   = 4.0; // Multiplier for fast elevator movement

    // Gripper 
    static constexpr double INITIAL_GRIPPER_CLOSE_ANGLE = 90.0;
    static constexpr double INITIAL_GRIPPER_OPEN_ANGLE =  0.0;
    static constexpr double INITIAL_ELEVATOR_MOTION_DURING_GRIPPER = 2.0; // [cm]
    static constexpr double GRIPPER_MOVEMENT_SPEED = 5.0; // degrees per button press



}