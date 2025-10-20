#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "Functions.h"
#include "Hardware.h"

#include <string>

#include <cmath>

class Oms
{
    public:
        Oms( Hardware * h ) : hardware{h}{}

        void elevator_set_height( double desired_height, bool async = true );
        void elevator_move_height( double delta_height , bool async = true );
        void oms_maintain_height();
        void SetGripper( double angle );
        void MoveGripper(int delta_angle);

        double cur_elevator_height = -1000;
        int cur_gripper_angle = constant::GRIPPER_CONTRACTED_ANGLE;

    private:

        Hardware * hardware;

        double elevator_target_height = constant::ELEVATOR_LIMIT_HIGH_HEIGHT;
        bool elevator_on_target = false;

};