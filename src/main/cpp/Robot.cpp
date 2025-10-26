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

    void autonomous_mode(){
        // put your autonomous code here
        move.autonomous_mode = true;
        set_running_lights(true);


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
    // end of simple tasks functions section


    int function_index = 0;
    int no_of_functions = 5;
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
                // Function 1
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
