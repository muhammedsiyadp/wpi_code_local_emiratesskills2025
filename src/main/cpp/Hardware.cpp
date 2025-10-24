/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#include "Hardware.h"

#define DEBUG true

Hardware::Hardware()
{
    ResetEncoders();
    ResetYaw();
}

void Hardware::ResetEncoders()
{
    LeftEncoder.Reset();
    BackEncoder.Reset();
    RightEncoder.Reset();
    ElevatorEncoder.Reset();
}

void Hardware::ResetYaw()
{
    navX.ZeroYaw();
}
void Hardware::SetLeft( double pwm ){
    LeftMotor.Set( pwm );
}
void Hardware::SetRight( double pwm ){
    RightMotor.Set( pwm );
}
void Hardware::SetBack( double pwm ){
    BackMotor.Set( pwm );
}
void Hardware::SetElevator( double pwm ){
    ElevatorMotor.Set( pwm );
}

void Hardware::Elevation( double angle, double elevator ){
    ElevatorMotor.Set( elevator );
    servo.SetAngle( angle );
}
void Hardware::SetGripper( double angle ){
    servo.SetAngle( angle );
}
void Hardware::SetRunningLED(bool on)
{
    runningLED.Set(on);
}

void Hardware::SetStoppedLED(bool on)
{
    stoppedLED.Set(on);
}

double Hardware::GetLeftEncoder()
{
    return LeftEncoder.GetRaw();
}

double Hardware::GetBackEncoder()
{
    return BackEncoder.GetRaw();
}

double Hardware::GetRightEncoder()
{
    return RightEncoder.GetRaw();
}

double Hardware::GetElevatorEncoder()
{
    return ElevatorEncoder.GetRaw();
}

double Hardware::GetYaw()
{
    return navX.GetYaw();
}

double Hardware::GetAngle()
{
    return navX.GetAngle();
}

bool Hardware::GetStopButton(){
    return stopButton.Get();
}
bool Hardware::GetStartButton(){
    return startButton.Get();
}

double Hardware::GetCobra( int channel ){
    return cobra.GetVoltage(channel);
}


double Hardware::getRightDistance( ){
    us_r.Ping();
    return us_r.GetRangeMM() / 10.0;
}

double Hardware::getLeftDistance( ){
    us_l.Ping();
    return us_l.GetRangeMM() / 10.0;
}

double Hardware::getFrontDistance_R_Raw( ){
    return (pow(right_sharp.GetAverageVoltage(), -1.2045)) * 27.726;
}

double Hardware::getFrontDistance_L_Raw( ){
    return (pow(left_sharp.GetAverageVoltage(), -1.2045)) * 27.726;
}
void Hardware::Update_sharp_sensors_background(){
    while (true){

        // we are using a simple low-pass filter to smooth the readings using exponential moving average
        double alpha = 0.2;
        double raw_L = getFrontDistance_L_Raw();
        double raw_R = getFrontDistance_R_Raw();

        filtered_sharp_sensor_L = alpha * raw_L + (1 - alpha) * filtered_sharp_sensor_L;
        filtered_sharp_sensor_R = alpha * raw_R + (1 - alpha) * filtered_sharp_sensor_R;
        delay(50);
    }
}
double Hardware::getFrontDistance_L(){
    return filtered_sharp_sensor_L;
}
double Hardware::getFrontDistance_R(){
    return filtered_sharp_sensor_R;
}






bool Hardware::GetLimitHigh(){
    //return limit_high.Get();
    return ElevatorMotor.GetLimitSwitch(0);
}

bool Hardware::GetLimitLow(){
    //return limit_low.Get();
    return ElevatorMotor.GetLimitSwitch(1);
}
double Hardware::GetBatteryVoltage(){
    return robotController.GetInputVoltage();
}