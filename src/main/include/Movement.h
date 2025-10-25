/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "Functions.h"
#include "Hardware.h"

#include <string>

#include <cmath>

class Movement
{
    public:
        Movement( Hardware * h ) : hardware{h}{}

        void InverseKinematics_local(double x_l, double y_l, double z);
        void global_to_local(double x_global , double y_global);
        void ForwardKinematics( double vl, double vr, double vb );
        void SetPosition( double x, double y, double th );
        
        void PositionDriver( double desired_x, double desired_y, double desired_th );
        void RotateToAngle( double desired_angle );
        void Rotate( double angle_deg );
        void DriveStraight(double distance_cm);
        void SideWalk(double distance_cm);


        void BackgroundTasks();
        void SetHeading(double th);
        void ReleaseHeading();



        double WheelSpeed( int encoder, double time );     // Returns the wheel speed based on the Encoder Ticks
        void ShuffleBoardUpdate();
        double get_x();
        double get_y();
        double get_th();

        void sensor_drive( double dist, std::string direction );
        void line_align( std::string direction );
        void align_to_wall(double distance_cm = 20);

        double desired_back_speed; 
        double desired_left_speed; 
        double desired_right_speed;

        
        bool maintain_heading_enabled = false;
        bool heading_on_target = false;
        
        double desired_vx_local;
        double desired_vy_local;
        double desired_vx_global;
        double desired_vy_global;
        double desired_vth;
        double desired_th;

        bool autonomous_mode = true;



    private:

        Hardware * hardware;

        double x_global  = 0;   //Robot Global Position on the X  axis  [cm]
        double y_global  = 0;   //Robot Global Position on the Y  axis  [cm]
        double th_global = 0;   //Robot Global Position on the Th axis  [degrees]

        int current_enc_l ;
        int previous_enc_l;

        int current_enc_r ;
        int previous_enc_r;

        int current_enc_b ;
        int previous_enc_b;

        const double linear_tolerance  = 2.0;    // [cm]
        const double angular_tolerance = 3.0;    // [degrees]

        const double max_linear_speed = 30.0;    // cm/s
        const double max_ang_speed = 0.75;       // rad/s

        double leftVelocity;
        double rightVelocity;
        double backVelocity; 


        double vx;
        double vy; 
        double vth;

        double offset_th;

};