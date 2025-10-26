/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: Muhammed siyad p
 * Last modification date: 26-10-2025
 * New version: 1.3.0.0

*************************************/

#include "Robot.h"

Robot r;
Hardware hard;
Movement move( &hard );
Oms oms( &hard );
OI oi;
Drive drive( &hard, &move, &oi, &oms );

void set_running_lights(bool running){
    hard.SetRunningLED(running);
    hard.SetStoppedLED(!running);
}

// section for add your autonomous functions
void start_to_court_midpoint(){
    move.SetPosition( 30, 35, 0 ); 
    oms.OpenGripper();
    move.PositionDriver(100,35,0);
    move.PositionDriver(100,100,0);
}
void from_court_midpoint_to_shelf_center(){
    move.PositionDriver(53,135,90);
}

void pick_from_shelf_and_goto_court_center(int r, int c){
    // Code to pick from shelf
    oms.OpenGripper();
    from_court_midpoint_to_shelf_center();
    //from shelf midpoint to first column
    if (r == 1){
        oms.elevator_set_height(6.7);
    }
    else if (r == 2){
        oms.elevator_set_height(18.5);
    }
    else if (r == 3){
        oms.elevator_set_height(30);
    }


    if (c == 1){
        move.PositionDriver(26,162,90);
        move.sensor_drive(16 , "front_l");
        move.sensor_drive(52 , "right");
        move.sensor_drive(16 , "front_l");
        move.SetPosition(26,162,90);
        move.line_align("right");
    }
    else if (c == 2){
        move.PositionDriver(26,162,90);
        move.sensor_drive(16 , "front_l");
        move.sensor_drive(52 , "right");
        move.sensor_drive(16 , "front_l");
        move.SetPosition(26,162,90);
        move.sensor_drive(37 , "right");
        move.line_align("right");
    }
    else if (c == 3){
        move.PositionDriver(74,162,90);
        move.sensor_drive(15 , "front_r");
        move.sensor_drive(53 , "left");
        move.sensor_drive(15 , "front_r");
        move.SetPosition(74,162,90);
        move.sensor_drive(46 , "left");
        move.line_align("left");
    }
    else if (c == 4){
        move.PositionDriver(74,162,90);
        move.sensor_drive(15 , "front_r");
        move.sensor_drive(53 , "left");
        move.sensor_drive(15 , "front_r");
        move.SetPosition(74,162,90);
        move.line_align("left");
    }
    while (oms.elevator_on_target == false){
        delay(100);
    }
    
    

    //pick and come to court centre
    move.DriveStraight(10);
    oms.CloseGripper();
    delay(1000);
    move.DriveStraight(-12);
    move.PositionDriver(100,100,0);


}
void place_on_stand(int r, int c){
    // Code to place at court center

    if (r == 1){
        oms.elevator_set_height(12);
    }
    else if (r == 2){
        oms.elevator_set_height(18.5);
    }
    else if (r == 3){
        oms.elevator_set_height(25);
    }

    if (c == 4){
        //center to stand to first row (right to left)
        move.PositionDriver(160,80,0);
        move.sensor_drive(50 , "right");
        move.sensor_drive(10 , "front_l");
        move.SetPosition(160,80,0);
        move.line_align("left");
    }
    else if (c == 3){
        move.PositionDriver(160,80,0);
        move.sensor_drive(50 , "right");
        move.sensor_drive(10 , "front_l");
        move.SetPosition(160,80,0);
        move.line_align("left");
        move.SideWalk(3);
        move.line_align("left");
    }
    else if (c ==2){
        move.PositionDriver(160,80,0);
        move.sensor_drive(50 , "right");
        move.sensor_drive(10 , "front_l");
        move.SetPosition(160,80,0);
        move.line_align("left");
        move.SideWalk(3);
        move.line_align("left");
        move.SideWalk(3);
        move.line_align("left");
    }
    else if (c == 1){
        move.PositionDriver(160,80,0);
        move.sensor_drive(50 , "right");
        move.sensor_drive(10 , "front_l");
        move.SetPosition(160,80,0);
        move.line_align("left");
        move.SideWalk(3);
        move.line_align("left");
        move.SideWalk(3);
        move.line_align("left");
        move.SideWalk(3);
        move.line_align("left");
        
    }

    while (oms.elevator_on_target == false){ //wait for elevator to reach target height
        delay(100);
    }

    //place and go back
    move.DriveStraight(11);
    oms.OpenGripper();
    delay(2000);
    move.DriveStraight(-13);
    move.PositionDriver(100,100,0);
    
}
void go_back_to_start(){
    // Code to return to start position
    move.PositionDriver(100,35,0);
    move.PositionDriver(30,35,0);
}
void autonomous_mode(){
    // put your autonomous code here
    move.autonomous_mode = true;
    set_running_lights(true);
    ////////////////////////////////////////////////////////////////////////////////////////////////
    start_to_court_midpoint();
    pick_from_shelf_and_goto_court_center(1, 1);
    place_on_stand(1, 3);
    pick_from_shelf_and_goto_court_center(2, 2);
    place_on_stand(1, 2);
    pick_from_shelf_and_goto_court_center(3, 3);
    place_on_stand(1, 1);
    go_back_to_start();
    ////////////////////////////////////////////////////////////////////////////////////////////////
    

    


    
    



}
// end of autonomous functions section

// section for adding your simple tasks functions
void simple_task_1(){
    // Code for simple task 1
}

void simple_task_2(){
    // Code for simple task 2
}

void simple_task_3(){
    // Code for simple task 3
}

int main() { 

    r.ds.Enable();
    

    std::thread oms_thread(&Oms::oms_maintain_height, &oms); // Start the elevator maintain height in a separate thread (background)
    std::thread movement_thread(&Movement::BackgroundTasks, &move); // Start the movement background tasks in a separate thread
    std::thread sharp_thread(&Hardware::Update_sharp_sensors_background, &hard); // Start the sharp sensors update in a separate thread
    delay(1000);


    
    // end of simple tasks functions section


    int function_index = 0;
    int no_of_functions = 6;
    frc::SmartDashboard::PutString("Function", "Autonomous Mode");
    while(1){
        if (!hard.GetResetButton()){
            function_index = (function_index + 1) % no_of_functions;
            if (function_index == 0){
                frc::SmartDashboard::PutString("Function", "Autonomous Mode");
            }
            else if (function_index == 1){
                frc::SmartDashboard::PutString("Function", "Teleop Drive Mode");
            }
            else if (function_index == 2){
                frc::SmartDashboard::PutString("Function", "Camera Mode");
            }
            else if (function_index == 3){
                frc::SmartDashboard::PutString("Function", "Simple Task 1");
            }
            else if (function_index == 4){
                frc::SmartDashboard::PutString("Function", "Simple Task 2");
            }
            else if (function_index == 5){
                frc::SmartDashboard::PutString("Function", "Simple Task 3");
            }
            delay(400);
        }
        if (!hard.GetStartButton()){
            delay(400);

            if (function_index == 0){ //run the autonomous function
                autonomous_mode();
            }
            else if (function_index == 1){ //run the manual drive function
                while (hard.GetStopButton()){
                    set_running_lights(true);
                    move.autonomous_mode = false;
                    drive.Execute();
                    delay(50);
                }
                set_running_lights(false);
            }
            else if (function_index == 2){ //run the camera function
                // Function 3
            }
            else if (function_index == 3){ //run the simple task one
                set_running_lights(true);
                simple_task_1();
                set_running_lights(false);
            }
            else if (function_index == 4){  //run the simple task two
                set_running_lights(true);
                simple_task_2();
                set_running_lights(false);
            }
            else if (function_index == 5){  //run the simple task three
                set_running_lights(true);
                simple_task_3();
                set_running_lights(false);
                // Function 6
            }

        }
        delay(200);
    
        
        
        

        
        
    }
    return 0; 
}
