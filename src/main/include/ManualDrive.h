/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#pragma once

#include "Hardware.h"
#include "Movement.h"
#include "OI.h"
#include "Oms.h"

#include <iostream>


class Drive
{
    public:
        Drive( Hardware * h, Movement * m, OI * o , Oms * om) : hardware{h}, move{m}, oi{o}, oms{om} {};
        void Execute();
    
    private:
        Movement * move;
        Hardware * hardware;
        OI* oi;
        Oms* oms;

        double inputLeftY = 0;
        double inputLeftX = 0;
        double inputRightY = 0;
        double inputRightX = 0;

        double prev_vx = 0;
        double prev_vy = 0;
        double prev_vth = 0;
        
        static constexpr double RAMP_UP = 5.0;

        static constexpr double RAMP_DOWN = 5.0;

        static constexpr double DELTA_LIMIT = 0.4;

        bool elevator_up_key_pressed = false;
        bool elevator_down_key_pressed = false;
        bool servo_left_pressed = false;
        bool servo_right_pressed = false;
        bool gripper_open_pressed = false;
        bool gripper_close_pressed = false;

        int pov_position = -1;

};