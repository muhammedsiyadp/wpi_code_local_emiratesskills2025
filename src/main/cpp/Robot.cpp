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
    delay(1000);

hard.SetRunningLED(false);
hard.SetStoppedLED(true);

while(1){
    // Wait until the Start Button is pressed
        
        //delay(50);
     while( hard.GetStartButton() ){delay(150); }

    hard.SetRunningLED(true);
    hard.SetStoppedLED(false);

    move.SetPosition( 0, 0, 0 ); 
     
    move.PositionDriver(100,100,0);
    delay(1000);
    move.PositionDriver(0,0,0);
   
    
    // //  move.PositionDriver(80,80,90);move.SetPosition( 0, 0, 0 ); 
    // //  move.PositionDriver(50,50,0);move.SetPosition( 0, 0, 0 ); 
    // //  move.PositionDriver(-50,-50,0);move.SetPosition( 0, 0, 0 ); 
    // //  move.PositionDriver(0,0,-90);move.SetPosition( 0, 0, 0 );
    // //  move.PositionDriver(70,0,0);move.SetPosition( 0, 0, 0 );
    // // move.PositionDriver(-80,-80,0);move.SetPosition( 0, 0, 0 );
    // // move.PositionDriver(-70,0,0);move.SetPosition( 0, 0, 0 );
    
        
    // // move.DriveStraight(100);
    //  //move.DriveStraight(-50);
    // // move.SideWalk(100);
    //  //move.SideWalk(-100);

     
    // oms.elevator_set_height(20);
    
    // oms.SetGripper(move.servo_current_angle);

    //  hard.SetRunningLED(false);
    //  hard.SetStoppedLED(true);
    move.autonomous_mode = false;
    while (true){
        drive.Execute();
        delay(50);
    }
     
    }
    return 0; 
}
