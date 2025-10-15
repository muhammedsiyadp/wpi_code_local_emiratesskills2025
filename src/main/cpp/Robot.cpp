/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#include "Robot.h"

int main() { 

// Start MockDS
    Robot r;
    r.ds.Enable();

    Hardware hard;
    Movement move( &hard );
    Oms oms( &hard );

    delay(1000);

hard.SetRunningLED(false);
hard.SetStoppedLED(true);

while(1){
    // Wait until the Start Button is pressed
        
        delay(50);
     while( hard.GetStartButton() ){ move.ShuffleBoardUpdate();delay(150); }

     hard.SetRunningLED(true);
     hard.SetStoppedLED(false);

     move.SetPosition( 30, 30, 0 ); 
     move.PositionDriver(200,100,180);
   
     /**
     move.PositionDriver(80,80,90);move.SetPosition( 0, 0, 0 ); 
     move.PositionDriver(50,50,0);move.SetPosition( 0, 0, 0 ); 
     move.PositionDriver(-50,-50,0);move.SetPosition( 0, 0, 0 ); 
     move.PositionDriver(0,0,-90);move.SetPosition( 0, 0, 0 );
     move.PositionDriver(70,0,0);move.SetPosition( 0, 0, 0 );
     move.PositionDriver(-80,-80,0);move.SetPosition( 0, 0, 0 );
      move.PositionDriver(-70,0,0);move.SetPosition( 0, 0, 0 );
       */
        
    // move.DriveStraight(100);
     //move.DriveStraight(-50);
    // move.SideWalk(100);
     //move.SideWalk(-100);

     
    

     hard.SetRunningLED(false);
     hard.SetStoppedLED(true);
     
}
    return 0; 
}
