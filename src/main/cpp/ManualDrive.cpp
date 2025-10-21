/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#include "ManualDrive.h"

void Drive::Execute()
{
    /**
     * Get Joystick Data
     */
    inputLeftX  = -oi->GetLeftDriveY();
    inputLeftY  = -oi->GetLeftDriveX();
    inputRightY = -oi->GetRightDriveY();
    inputRightX = -oi->GetRightDriveX();


    elevator_up_key_pressed = oi->GetDriveRightTrigger();
    elevator_down_key_pressed = oi->GetDriveLeftTrigger();
    servo_left_pressed = oi->GetDriveLeftBumper();
    servo_right_pressed = oi->GetDriveRightBumper();
    gripper_close_pressed = oi->GetDriveCircleButton();
    gripper_open_pressed = oi->GetDriveSquareButton();


    pov_position = oi->GetDrivePOV();


    
    double vx = 0;
    double vy = 0;
    double vth = 0;

    double xy_speed = 40;   // [cm/s]
    double th_speed = 1;    // [rad/s]

    if(inputLeftX >= DELTA_LIMIT){
        vx = xy_speed;}
    else if (inputLeftX <= -DELTA_LIMIT){
        vx = -xy_speed;}
    
    if(inputLeftY >= DELTA_LIMIT){
        vy = xy_speed;}
    else if (inputLeftY <= -DELTA_LIMIT){
        vy = -xy_speed;}
    if(inputRightX >= DELTA_LIMIT){
        vth = th_speed;}
    else if (inputRightX <= -DELTA_LIMIT){
        vth = -th_speed;}

    if ( (vx - prev_vx) > 0 )       { vx  = prev_vx  + RAMP_UP;   }
    else if ( (vx - prev_vx) < 0 )  { vx  = prev_vx  - RAMP_DOWN; }
    if ( (vy - prev_vy) > 0 )       { vy  = prev_vy  + RAMP_UP;   }
    else if ( (vy - prev_vy) < 0 )  { vy  = prev_vy  - RAMP_DOWN; }
    if ( (vth - prev_vth) > 0 )     { vth = prev_vth + RAMP_UP / 50;   }
    else if ( (vth - prev_vth) < 0 ){ vth = prev_vth - RAMP_DOWN / 50; }

    if (inputLeftX == 0){
        vx = 0;
    }
    if (inputLeftY == 0){
        vy = 0;
    }
    if (inputRightX == 0){
        vth = 0;
    }


    std::clamp( vx,  xy_speed, -xy_speed );
    std::clamp( vy,  xy_speed, -xy_speed );
    std::clamp( vth, th_speed, -th_speed );

    move->InverseKinematics(vx, vy, vth);       // Based on the desired vx, vy and vth calculates the PWM to be applied to each wheel

    hardware->SetLeft ( move->desired_left_speed );
    hardware->SetBack ( move->desired_back_speed );
    hardware->SetRight( move->desired_right_speed );

    prev_vx = vx;
    prev_vy = vy;
    prev_vth = vth;

    // std::cout << "vx: " << vx << " vy: " << vy << " vth: " << vth << std::endl;
    std::cout << "inputLeftY: " << inputLeftY << " inputLeftX: " << inputLeftX << " inputRightY: " << inputRightY << " inputRightX: " << inputRightX << std::endl;

    //code for OMS
    if (elevator_up_key_pressed ){
        oms->elevator_move_height(constant::ELEVATOR_MOVE_SPEED, true);
    }
    else if (elevator_down_key_pressed ){
        oms->elevator_move_height(-constant::ELEVATOR_MOVE_SPEED, true);
    }
    if (servo_left_pressed ){
        oms->MoveGripper(constant::GRIPPER_MOVEMENT_SPEED);
    }
    else if (servo_right_pressed ){
        oms->MoveGripper(-constant::GRIPPER_MOVEMENT_SPEED);
    }

    if (pov_position == 0){ // Up
        oms->elevator_move_height(constant::ELEVATOR_MOVE_SPEED, false);
    }
    else if (pov_position == 180){ // Down
        oms->elevator_move_height(-constant::ELEVATOR_MOVE_SPEED, false);
    }
    if (gripper_close_pressed){
        oms->CloseGripper();
    }
    else if (gripper_open_pressed){
        oms->OpenGripper();
    }




}   
