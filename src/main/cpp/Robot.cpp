/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: Muhammed siyad p
 * Last modification date: 22-10-2025
 * New version: 1.2.1.0

*************************************/

#include "Robot.h"

int main() { 

// Start MockDS
    Robot r;
    r.ds.Enable();

    Hardware hard;
    Movement move( &hard );
    Oms oms( &hard );
    OI oi;
    Drive drive( &hard, &move, &oi, &oms );

    std::thread oms_thread(&Oms::oms_maintain_height, &oms); // Start the elevator maintain height in a separate thread (background)
    std::thread movement_thread(&Movement::BackgroundTasks, &move); // Start the movement background tasks in a separate thread
    std::thread sharp_thread(&Hardware::Update_sharp_sensors_background, &hard); // Start the sharp sensors update in a separate thread
    delay(1000);

hard.SetRunningLED(false);
hard.SetStoppedLED(true);

while(1){
    // Wait until the Start Button is pressed
        
        //delay(50);
     while( hard.GetStartButton() ){delay(150); }

    

    hard.SetRunningLED(true);
    hard.SetStoppedLED(false);

    //move.line_align("right");
    
    
    //from start to midpoint of court
    move.SetPosition( 30, 35, 0 ); 
    oms.OpenGripper();
    move.PositionDriver(100,35,0);
    move.PositionDriver(100,100,0);
    
    //from court midpoint to shelf midpoint
    move.PositionDriver(53,135,90);

    oms.elevator_set_height(6.7);

    //from shelf midpoint to first column
    move.PositionDriver(26,162,90);
    move.sensor_drive(16 , "front_l");
    move.sensor_drive(52 , "right");
    move.sensor_drive(16 , "front_l");
    move.SetPosition(26,162,90);
    move.line_align("right");


    //pick and come to court centre
    move.DriveStraight(10);
    oms.CloseGripper();
    delay(1000);
    move.DriveStraight(-12);
    move.PositionDriver(100,100,0);

    oms.elevator_set_height(12);

    //center to stand to first row (right to left)
    move.PositionDriver(160,80,0);
    move.sensor_drive(50 , "right");
    move.sensor_drive(10 , "front_l");
    move.SetPosition(160,80,0);
    move.line_align("left");

    //place and go back
    move.DriveStraight(11);
    oms.OpenGripper();
    delay(2000);
    move.DriveStraight(-13);
    move.PositionDriver(100,100,0);

    oms.elevator_set_height(18.5);

    //from court midpoint to shelf midpoint
    move.PositionDriver(53,135,90);

    //from shelf midpoint to first column
    move.PositionDriver(26,162,90);
    move.sensor_drive(16 , "front_l");
    move.sensor_drive(52 , "right");
    move.sensor_drive(16 , "front_l");
    move.SetPosition(26,162,90);
    move.line_align("right");

     //pick and come to court centre
    move.DriveStraight(10);
    oms.CloseGripper();
    delay(1000);
    move.DriveStraight(-12);
    move.PositionDriver(100,100,0);

    oms.elevator_set_height(18.5);

    //center to stand to first row (right to left)
    move.PositionDriver(160,80,0);
    move.sensor_drive(50 , "right");
    move.sensor_drive(10 , "front_l");
    move.SetPosition(160,80,0);
    move.line_align("left");

    //place and go back
    move.DriveStraight(11);
    oms.OpenGripper();
    delay(2000);
    move.DriveStraight(-13);
    move.PositionDriver(100,100,0);




    


    delay(3000);

    
    while (false){
        move.autonomous_mode = false;
        drive.Execute();
        delay(50);
    }
     
    }
    return 0; 
}
