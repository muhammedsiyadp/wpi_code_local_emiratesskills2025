#include "Oms.h"

void Oms::elevator_set_height( double desired_height , bool async ){
    desired_height = std::clamp( desired_height, (double)constant::ELEVATOR_LIMIT_LOW_HEIGHT, (double)constant::ELEVATOR_LIMIT_HIGH_HEIGHT );
    elevator_target_height = desired_height;
    elevator_on_target = false;
    if (!async)
    {
        while ( !elevator_on_target )
        {
            delay(100);
        }
    }

}
void Oms::elevator_move_height( double delta_height , bool fast , bool async ){
    if (fast) elevator_set_height( cur_elevator_height + delta_height * constant::ELEVATOR_FAST_MOVE_FACTOR, async );
    else      elevator_set_height( cur_elevator_height + delta_height  ,async );
}

void Oms::oms_maintain_height(){
    double delta_time = 0.2; // [s]
    double current_enc  = hardware->GetElevatorEncoder();
    double previous_enc = hardware->GetElevatorEncoder();

    while(true) {

        current_enc = hardware->GetElevatorEncoder();
        double delta_enc = current_enc - previous_enc;
        previous_enc = current_enc;

        // Speed per iteration
        double elevatorVelocity = (((2 * M_PI * ( 1.25 ) * delta_enc) / (constant::PULSE_PER_REV * delta_time)));   // [cm/s]

        double delta_elev = elevatorVelocity * delta_time;  // Displacement per iteration

        cur_elevator_height = cur_elevator_height + delta_elev;

        if( !hardware->GetLimitHigh() ){ cur_elevator_height = constant::ELEVATOR_LIMIT_HIGH_HEIGHT; }
        //if( !hardware->GetLimitLow()  ){ cur_elevator_height = 10; }


        double elev_diff = elevator_target_height - cur_elevator_height;

        double max_linear_speed = constant::ELEVATOR_MAX_SPEED; // cm/s

        double desired_v = (elev_diff / 5.0) * max_linear_speed; // [cm/s]
        if     ( desired_v >  max_linear_speed ){ desired_v =  max_linear_speed; }
        else if( desired_v < -max_linear_speed ){ desired_v = -max_linear_speed; }

        if( abs(elev_diff) < constant::ELEVATOR_POSITION_TOLERANCE )  { 
            elevator_on_target = true; 
            desired_v = 0; 
        }


        if ( !hardware->GetStopButton() ){  // Stop the Motors when the Stop Button is pressed
            hardware->SetElevator ( 0 );
        }else{
            hardware->SetElevator ( desired_v / 70.0 );
        }

        frc::SmartDashboard::PutNumber("cur_elevator_height",  cur_elevator_height );
        frc::SmartDashboard::PutNumber("Battery Voltage",  hardware->GetBatteryVoltage() );


        delay( delta_time * 1000 );


    }
    
    hardware->SetElevator ( 0 );
}
void Oms::SetGripper( double angle ){
    cur_gripper_angle = angle;
    std::clamp( cur_gripper_angle, 0, 300 );
    frc::SmartDashboard::PutNumber("Servo Pos", cur_gripper_angle );
    hardware->SetGripper( cur_gripper_angle );
}
void Oms::MoveGripper(int delta_angle){
    cur_gripper_angle += delta_angle;
    cur_gripper_angle = std::clamp( cur_gripper_angle, 0, 300 );
    frc::SmartDashboard::PutNumber("Servo Pos", cur_gripper_angle );
    hardware->SetGripper( cur_gripper_angle );
}