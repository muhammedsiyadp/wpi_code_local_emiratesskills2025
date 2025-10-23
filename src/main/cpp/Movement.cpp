/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#include "Movement.h"

// All the backgroud tasks related to movement will be defined here
// this include maintain yaw heading, update encoders, global coordinates and more.
void Movement::BackgroundTasks() {
    double delta_time = 0.1; // [s]

    previous_enc_l = hardware->GetLeftEncoder();
    previous_enc_r = hardware->GetRightEncoder();
    previous_enc_b = hardware->GetBackEncoder();
    delay(100);

    while (true) {

        current_enc_l = hardware->GetLeftEncoder();
        double delta_enc_l = current_enc_l - previous_enc_l;
        previous_enc_l = current_enc_l;

        current_enc_r = hardware->GetRightEncoder();
        double delta_enc_r = current_enc_r - previous_enc_r;
        previous_enc_r = current_enc_r;

        current_enc_b = hardware->GetBackEncoder();
        double delta_enc_b = current_enc_b - previous_enc_b;
        previous_enc_b = current_enc_b;


        //Wheels Velocity
        leftVelocity  = WheelSpeed(delta_enc_l, delta_time );     // [cm/s]
        rightVelocity = WheelSpeed(delta_enc_r, delta_time );     // [cm/s]
        backVelocity  = WheelSpeed(delta_enc_b, delta_time );     // [cm/s]
        
        //Forward Kinematics
        ForwardKinematics( leftVelocity, rightVelocity, backVelocity );

        //Robot Displacement
        double delta_x  = vx  * delta_time;
        double delta_y  = vy  * delta_time;
        double delta_th = vth * delta_time;

        //Robot Position update
        x_global  = x_global  + delta_x;
        y_global  = y_global  + delta_y;
        // th_global = th_global + ((delta_th / M_PI) * 180); 

        th_global = -hardware->GetYaw() - offset_th;            // Angle based on the Gyro

        if      ( th_global <  0  ) { th_global = th_global + 360; }
        else if ( th_global > 360 ) { th_global = th_global - 360; } 


        //code to maintain heading
        if (maintain_heading_enabled) {
            double th_diff = desired_th - th_global;
            if      ( th_diff < -180 ) { th_diff = th_diff + 360; }
            else if ( th_diff >  180 ) { th_diff = th_diff - 360; }
            desired_vth = (th_diff / 10.0) * max_ang_speed;  // [rad/s]
            if     ( desired_vth >  max_ang_speed ){ desired_vth =  max_ang_speed; }
            else if( desired_vth < -max_ang_speed ){ desired_vth = -max_ang_speed; }
            if( abs(th_diff) < angular_tolerance ){ 
                desired_vth = 0;
                heading_on_target = true;
            }
            else {
                heading_on_target = false;
            }

        }

        
        if (autonomous_mode){
            InverseKinematics( desired_vx, desired_vy, desired_vth );
            if ( !hardware->GetStopButton() ){  // Stop the Motors when the Stop Button is pressed
                hardware->SetLeft ( 0 );
                hardware->SetBack ( 0 );
                hardware->SetRight( 0 );
                // break;
            }else{
                hardware->SetLeft ( desired_left_speed );
                hardware->SetBack ( desired_back_speed );
                hardware->SetRight( desired_right_speed );
            }
        }
        
        ShuffleBoardUpdate();
        delay( delta_time * 1000 ); 
    }
}

void Movement::PositionDriver( double desired_x, double desired_y, double temp_desired_th ) {
    double temp_th_diff ;
    do{

        double desired_position[3] = { desired_x, desired_y, temp_desired_th };   // [cm], [cm], [degrees]
        double current_position[3] = { x_global , y_global, th_global };     // [cm], [cm], [degrees]

        double x_diff  = desired_position[0] - current_position[0];
        double y_diff  = desired_position[1] - current_position[1];
                                

        double temp_desired_vx = (x_diff / 10.0) * max_linear_speed; // [cm/s]
        if     ( temp_desired_vx >  max_linear_speed ){ temp_desired_vx =  max_linear_speed; }
        else if( temp_desired_vx < -max_linear_speed ){ temp_desired_vx = -max_linear_speed; }
        desired_vx = temp_desired_vx;

        double temp_desired_vy = (y_diff / 10.0) * max_linear_speed; // [cm/s]
        if     ( temp_desired_vy >  max_linear_speed ){ temp_desired_vy =  max_linear_speed; }
        else if( temp_desired_vy < -max_linear_speed ){ temp_desired_vy = -max_linear_speed; }
        desired_vy = temp_desired_vy;
        desired_th = temp_desired_th;

        


        if( abs(x_diff) < linear_tolerance )  { desired_vx = 0;  }
        if( abs(y_diff) < linear_tolerance )  { desired_vy = 0;  }
        delay(200);

    }while( desired_vx != 0 || desired_vy != 0 || !heading_on_target );

    desired_vx = 0;
    desired_vy = 0;
    desired_vth = 0;
    desired_left_speed  = 0;
    desired_right_speed = 0;
    desired_back_speed  = 0;

    delay(250);
}

void Movement::RotateToAngle( double desired_angle ){
    maintain_heading_enabled = true;
    desired_th = desired_angle;
    do{
        delay(200);

    }while(!heading_on_target);

    desired_vth = 0;
    desired_left_speed  = 0;
    desired_right_speed = 0;
    desired_back_speed  = 0;
    delay(250);
}
void Movement::Rotate( double angle_deg ){
    double target_angle = th_global + angle_deg;
    if      ( target_angle <  0  ) { target_angle = target_angle + 360; }
    else if ( target_angle > 360 ) { target_angle = target_angle - 360; } 
    RotateToAngle( target_angle );
}
void Movement::InverseKinematics(double x, double y, double z){

    double th_radius = th_global * ( M_PI / 180.0 );

    // From Global to Local
    double x_l = x * cos( -th_radius ) - y * sin( -th_radius );
    double y_l = x * sin( -th_radius ) + y * cos( -th_radius );
    
    desired_back_speed  = (( x_l * cos( M_PI*( 90.0/180.0))) + ( -y_l * sin(M_PI*( 90.0/180.0))) + (z * constant::FRAME_RADIUS) );
    desired_left_speed  = (( x_l * cos( M_PI*(210.0/180.0))) + ( -y_l * sin(M_PI*(210.0/180.0))) + (z * constant::FRAME_RADIUS) );
    desired_right_speed = (( x_l * cos( M_PI*(330.0/180.0))) + ( -y_l * sin(M_PI*(330.0/180.0))) + (z * constant::FRAME_RADIUS) );

    desired_back_speed  = -desired_back_speed  / 70.0;   // cm/s to PWM [0-1]
    desired_left_speed  = -desired_left_speed  / 55.0;   // cm/s to PWM [0-1]    
    desired_right_speed = -desired_right_speed / 55.0;   // cm/s to PWM [0-1]
   
}

void Movement::ForwardKinematics( double vl, double vr, double vb ){
    
    double vx_l   = ( (      0     * vb ) + ( (1.0/sqrt(3.0)) * vr) + ( (-1.0/sqrt(3.0)) * vl) );   // [cm/s]
    double vy_l   = ( ( (-2.0/3.0) * vb ) + (    (1.0/3.0)    * vr) + (     (1.0/3.0)    * vl) );   // [cm/s]
    double vth_l  = ( ( ( 1.0/3.0) * vb ) + (    (1.0/3.0)    * vr) + (     (1.0/3.0)    * vl) ) / constant::FRAME_RADIUS ; // [rad/s]

    double th_radius = th_global * ( M_PI / 180.0 );

    // From Local to Global
    vx = vx_l * cos( th_radius ) - vy_l * sin( th_radius );
    vy = vx_l * sin( th_radius ) + vy_l * cos( th_radius );
    vth = vth_l;
}

double Movement::WheelSpeed( int encoder, double time ){
        double speed  = -1 * (((2 * M_PI * constant::WHEEL_RADIUS * encoder) / (constant::PULSE_PER_REV * time)));   // [cm/s]

        if ( time == 0 ) { speed  = 0; }

        return speed;
}

void Movement::SetPosition( double x, double y, double th ){
    x_global  = x;
    y_global  = y;
    SetHeading( th );
}

void Movement::SetHeading( double th ){
    offset_th = -hardware->GetYaw() - th;
    th_global = th;
    desired_th = th;
    maintain_heading_enabled = true;

}
void Movement::ReleaseHeading(){
    desired_vth = 0;
    maintain_heading_enabled = false;
}

void Movement::ShuffleBoardUpdate(){

    frc::SmartDashboard::PutNumber("desired_back_speed",  desired_back_speed );
    frc::SmartDashboard::PutNumber("desired_left_speed",  desired_left_speed );
    frc::SmartDashboard::PutNumber("desired_right_speed", desired_right_speed);

    frc::SmartDashboard::PutNumber("robot_x",  x_global );
    frc::SmartDashboard::PutNumber("robot_y",  y_global );
    frc::SmartDashboard::PutNumber("robot_th", th_global);

    frc::SmartDashboard::PutNumber("vx",  vx );
    frc::SmartDashboard::PutNumber("vy",  vy );
    frc::SmartDashboard::PutNumber("vth", vth);

    frc::SmartDashboard::PutBoolean("Stop Button",  hardware->GetStopButton() );

    frc::SmartDashboard::PutNumber("desired_vx",  desired_vx );
    frc::SmartDashboard::PutNumber("desired_vy",  desired_vy );
    frc::SmartDashboard::PutNumber("desired_vth", desired_vth );

    frc::SmartDashboard::PutNumber("Encoder_Left",  hardware->GetLeftEncoder() );
    frc::SmartDashboard::PutNumber("Encoder_Right", hardware->GetRightEncoder() );
    frc::SmartDashboard::PutNumber("Encoder_Back",  hardware->GetBackEncoder() );

    frc::SmartDashboard::PutNumber("Left Ultrasonic",  hardware->getLeftDistance( ) );
    frc::SmartDashboard::PutNumber("Right Ultrasonic",  hardware->getRightDistance( ) );
    frc::SmartDashboard::PutNumber("Right SharpIR",  hardware->getFrontDistance_R( ) );
    frc::SmartDashboard::PutNumber("Left SharpIR",  hardware->getFrontDistance_L( ) );

    frc::SmartDashboard::PutNumber("Left Cobra",  hardware->GetCobra(0) );
    frc::SmartDashboard::PutNumber("Left_c Cobra",  hardware->GetCobra(1) );
    frc::SmartDashboard::PutNumber("Right_c Cobra",  hardware->GetCobra(2) );
    frc::SmartDashboard::PutNumber("Right Cobra",  hardware->GetCobra(3) );

    frc::SmartDashboard::PutNumber("Limit High",  hardware->GetLimitHigh() );
    frc::SmartDashboard::PutNumber("Limit Low",  hardware->GetLimitLow() );
    frc::SmartDashboard::PutNumber("desired th",  desired_th );

}

double Movement::get_x() { return x_global;  }

double Movement::get_y() { return y_global;  }

double Movement::get_th(){ return th_global; }



void Movement::sensor_drive( double dist, std::string direction ){

    double sensor_reading = 0;

    while( sensor_reading > dist + 2 || sensor_reading < dist - 2 ){

        if      ( direction.compare( "front" ) == 0 ){
            sensor_reading = hardware->getFrontDistance_R();

            if( sensor_reading > dist ){
                InverseKinematics(  20, 0, 0 );
            }else{
                InverseKinematics( -20, 0, 0 );
            }
        }else if( direction.compare( "left" ) == 0 ){
            sensor_reading = hardware->getLeftDistance();

            if( sensor_reading > dist ){
                InverseKinematics( 0,  20, 0 );
            }else{
                InverseKinematics( 0, -20, 0 );
            }
        }else if( direction.compare( "right" ) == 0 ){
            sensor_reading = hardware->getRightDistance();

            if( sensor_reading > dist ){
                InverseKinematics( 0, -20, 0 );
            }else{
                InverseKinematics( 0,  20, 0 );
            }
        }

        if ( !hardware->GetStopButton() ){  // Stop the Motors when the Stop Button is pressed
            hardware->SetLeft ( 0 );
            hardware->SetBack ( 0 );
            hardware->SetRight( 0 );
            // break;
        }else{
            hardware->SetLeft ( desired_left_speed );
            hardware->SetBack ( desired_back_speed );
            hardware->SetRight( desired_right_speed );
        }

        ShuffleBoardUpdate();

        delay( 50 );

    }

    hardware->SetLeft ( 0 );
    hardware->SetBack ( 0 );
    hardware->SetRight( 0 );

    delay( 150 );
}

void Movement::line_align( std::string direction ){

    frc::SmartDashboard::PutString("Process",  "Cobra Align" );

    bool cobra_l  = false;
    bool cobra_r  = false;
    bool cobra_cl = false;
    bool cobra_cr = false;


    while( !cobra_cl || !cobra_cr ){
        
        cobra_l  = hardware->GetCobra(0) > 2.5;
        cobra_r  = hardware->GetCobra(3) > 2.5;
        cobra_cl = hardware->GetCobra(1) > 2.5;
        cobra_cr = hardware->GetCobra(2) > 2.5;


        if      ( direction.compare( "left" ) == 0){
            InverseKinematics( 0,  20, 0 );  
        }else if( direction.compare( "right" ) == 0 ){
            InverseKinematics( 0, -20, 0 );  
        }else{
            break;
        }

        
        if ( !hardware->GetStopButton() ){  // Stop the Motors when the Stop Button is pressed
            hardware->SetLeft ( 0 );
            hardware->SetBack ( 0 );
            hardware->SetRight( 0 );
            // break;
        }else{
            hardware->SetLeft ( desired_left_speed );
            hardware->SetBack ( desired_back_speed );
            hardware->SetRight( desired_right_speed );
        }

        delay(50);
    }
    hardware->SetLeft ( 0 );
    hardware->SetBack ( 0 );
    hardware->SetRight( 0 );

    delay(200);
}

void Movement::DriveStraight(double distance_cm) {
    // Positive distance = forward, negative = backward
    double current_x = x_global;
    double current_y = y_global;
    double target_th = th_global;

    // Compute target position in the direction the robot is facing
    double angle_rad = target_th * (M_PI / 180.0);
    double target_x = current_x + distance_cm * cos(angle_rad);
    double target_y = current_y + distance_cm * sin(angle_rad);

    // Call PositionDriver to handle encoder-based motion
    PositionDriver(target_x, target_y, target_th);
}

void Movement::SideWalk(double distance_cm) {
    // Positive distance = right, negative = left
    double current_x = x_global;
    double current_y = y_global;
    double target_th = th_global;

    // Convert heading to radians
    double angle_rad = target_th * (M_PI / 180.0);

    // Compute sideways displacement
    double target_x = current_x + distance_cm * sin(angle_rad);
    double target_y = current_y - distance_cm * cos(angle_rad);

    // Call PositionDriver to move sideways
    PositionDriver(target_x, target_y, target_th);
}

void Movement::CorrectHeadingUsingFrontSensors() {
    double left = hardware->getFilteredFrontDistance_L();
    double right = hardware->getFilteredFrontDistance_R();
    double diff = left - right;

    // Log values

    if (fabs(diff) > 0.5) {
        double correction_speed = 0.2;  // tune this value
        if (diff > 0) {
            // Left side farther → rotate slightly left
            InverseKinematics(0, 0, correction_speed);
        } else {
            // Right side farther → rotate slightly right
            InverseKinematics(0, 0, -correction_speed);
        }

        // Apply correction briefly
        hardware->SetLeft (desired_left_speed);
        hardware->SetBack (desired_back_speed);
        hardware->SetRight(desired_right_speed);

        delay(100); // brief correction
        hardware->SetLeft (0);
        hardware->SetBack (0);
        hardware->SetRight(0);
    }
}



