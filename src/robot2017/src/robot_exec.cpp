#include "robot_exec.h"
#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_msgs/MotorFeedback.h"
#include "robot_msgs/Status.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>

// TODO slow lift speed between 0-5 deg
// TODO also look into reducing deadzone
// TODO prevent raising storage when lift is up

const char* motorPath = "/dev/ttyO1";  // Connected via UART
// const char* motorPath = "/dev/ttyUSB0" // Connected via USB
const float motorFastScale = 1.0f;
const float motorSlowScale = 0.5f;

const int   liftSpeedSlow    = 5000;  // RPM
const int   liftSpeedFast    = 10000; // RPM
const float liftBrakeCurrent = 8.0f;
const int   storageSpeedSlow = 12000; // RPM
const int   storageSpeedFast = 25000; // RPM

// Actual limits are up:0, down:180
const int liftDownLimit    = 175;
const int liftUpLimit      = 4;
// Lift must be below this for storage to move
const int storageLiftLimit = 105;

RobotExec::RobotExec(bool onPC, bool debug, bool autoActive)
    : dead(true), onPC(onPC), debug(debug), autonomyActive(autoActive),
      leftRatio(0.0f), rightRatio(0.0f), sensors(onPC), prevState(false), limitOverride(false),
      LeftDrive(LEFTDRIVE,   Alien_4260),
      RightDrive(RIGHTDRIVE, Alien_4260),
      Lift(LIFT,             Alien_4260),
      Storage(STORAGE,       Alien_4260),
      Bucket(BUCKET,         Alien_4260)
{
    if(!onPC) {
        BLDC::init((char*) motorPath);
        sensors.setStatusLed(Sensors::READY);
    }
}

void RobotExec::teleopReceived(const robot_msgs::Teleop& cmd)
{
    //if debugging, ignore teleop cmds
    if (this->debug) {
        std::stringstream message;

        message << "Teleop Command recieved: ";
        message << " " << +cmd.a << " " << +cmd.b << " " << +cmd.x << " " << +cmd.y;
        message << " " << +cmd.lb << " " << +cmd.rb << " " << +cmd.back;
        message << " " << +cmd.start << " " << +cmd.l_thumb << " " << +cmd.r_thumb;
        message << " (" << +cmd.x_l_thumb << " " << +cmd.y_l_thumb << ")";
        message << " (" << +cmd.x_r_thumb << " " << +cmd.y_r_thumb << ")";
        message << " (" << +cmd.l_trig << " " << +cmd.r_trig << ")";

        ROS_DEBUG_STREAM(message.str());
    }

    //if autonomy mode is toggled (start button is pressed)
    modeTransition(cmd.start);

    if(isAutonomyActive())
        return;
    else
    {
        //function for teleop
        teleopExec(cmd);
    }

    // byte range should be [-100, 100]
    //driveLeft.set(cmd.y_l_thumb * 1.0f / 100.0f);
    //driveRight.set(cmd.y_r_thumb * 1.0f / 100.0f);
}

void RobotExec::autonomyReceived(const robot_msgs::Autonomy& cmd)
{
    std::stringstream message;

    message << "Autonomy Command recieved: L:" << cmd.leftRatio << " R:"
    << cmd.rightRatio << " Storage:" << (cmd.storageUp ? "up" : (cmd.storageDown ? "down" : "-"))
    << " Lift:" << (cmd.liftUp ? "up" : (cmd.liftDown ? "down" : "-"))
    << " Drum:" << cmd.drumRatio;

    ROS_DEBUG_STREAM_COND(this->isDebugMode(),message.str());

    if(this->autonomyActive)
    {
        autonomyExec(cmd);
    }
}

//adds debouncing for start button
//ensures that if button shows up as pressed for several messages in a row
//it will not keep toggling the autonomy active state
//mode will not toggle until there is at least one unpressed button in msg
//in between two messages with button pressed
void RobotExec::modeTransition(const bool buttonState)
{
    if (prevState == false && buttonState == true)
    {
        setAutonomyActive(!isAutonomyActive()); //toggle mode
        prevState = true;

        ROS_DEBUG_STREAM("Changed autonomy state to " << isAutonomyActive());
        enable.data = isAutonomyActive(); //toggle data in autonomy enable message
    }
    else if (prevState == true && buttonState == false)
    {
        prevState = false;
    }
    
    //do nothing if both true or both false
}

// ==== TELEOP CONTROLS ====
// Driving:  left stick Y, right stick X
// Drum:     trigger L (dump), trigger R (dig)
// Lift:     button Y (up), button A (down)
// Storage:  button B (up), button X (down)
// Autonomy: button Start (toggle)
//  - autonomy toggle control is in teleopReceived
// Deadman:  L bumper (hold)
// Slow:     R bumper (hold)
//  - only applies to lift and drive
// Limits:   Select
//  - for overriding limits
// ==== MOTOR CONTROLS ====
// Driving:  duty cycle (% voltage, -1.0 to 1.0)
// Drum:     duty cycle (% voltage, -1.0 to 1.0)
// Lift:     speed (raw RPM)
// Storage:  speed (raw RPM)
void RobotExec::teleopExec(const robot_msgs::Teleop& cmd)
{
    dead = !cmd.lb;
    ROS_DEBUG_STREAM("Robot killed: " << dead);
    if(dead)
    {
        killMotors();
        sensors.setStatusLed(Sensors::READY);
        return;
    }
    else
    {
        sensors.setStatusLed(Sensors::ACTIVE);
    }

    // DRIVING
    // Note: y axis is given as negative = up, positive = down
    //       x axis also needs to be reversed when going backwards
    if(cmd.y_l_thumb == 0)
    {
        // special case for spins
        rightRatio = -cmd.x_r_thumb;
        leftRatio  =  cmd.x_r_thumb;
    }
    else if(cmd.y_l_thumb < 0)
    {
        if(cmd.x_r_thumb > 0)
        {
            rightRatio = -cmd.y_l_thumb - cmd.x_r_thumb;
            leftRatio = fmax(-cmd.y_l_thumb, cmd.x_r_thumb);
        }
        else
        {
            rightRatio = fmax(-cmd.y_l_thumb, -cmd.x_r_thumb);
            leftRatio = -cmd.y_l_thumb + cmd.x_r_thumb;
        }
    }
    else
    {
        if(cmd.x_r_thumb < 0)
        {
            rightRatio = -fmax(-cmd.y_l_thumb, -cmd.x_r_thumb);
            leftRatio = -cmd.y_l_thumb - cmd.x_r_thumb;
        }
        else
        {
            rightRatio = -cmd.y_l_thumb + cmd.x_r_thumb;
            leftRatio = -fmax(cmd.y_l_thumb, cmd.x_r_thumb);
        }
    }

    if(!this->onPC) {
        if(cmd.rb)
        {
            leftRatio  *= motorSlowScale;
            rightRatio *= motorSlowScale;
        }
        else
        {
            leftRatio  *= motorFastScale;
            rightRatio *= motorFastScale;
        }
        LeftDrive.set_Duty(leftRatio);
        RightDrive.set_Duty(rightRatio);
    }
    std::stringstream msg;
    msg << "Left Ratio " << leftRatio << ", Right Ratio " << rightRatio;
    ROS_DEBUG_STREAM(msg.str());

    // BUCKET DRUM
    // positive = dig, negative = dump
    if(cmd.l_trig > 0.0f)
    {
        ROS_DEBUG_STREAM_COND(this->isDebugMode(), "ENTERED DUMPING STATE");
        Bucket.set_Duty(-cmd.l_trig);
    }
    else if(cmd.r_trig > 0.0f)
    {
        ROS_DEBUG_STREAM_COND(this->isDebugMode(), "ENTERED DIG STATE");
        Bucket.set_Duty(cmd.r_trig);
    }
    else
    {
        Bucket.set_Duty(0.0f);
    }

    // DRUM LIFT
    // positive = down, negative = up
    int teleopLiftSpeed = (cmd.rb ? liftSpeedSlow : liftSpeedFast);
    if(cmd.y && checkLimit(DIR_UP, ARM_LIFT)) {
        lastLiftDir = DIR_UP;
        Lift.set_Speed(-teleopLiftSpeed);
    }
    else if(cmd.a && checkLimit(DIR_DOWN, ARM_LIFT))
    {
        lastLiftDir = DIR_DOWN;
        Lift.set_Speed(teleopLiftSpeed);
    }
    else
    {
        Lift.apply_Brake(liftBrakeCurrent);
    }

    // SECONDARY STORAGE
    // positive = down, negative = up
    int teleopStorageSpeed = (cmd.rb ? storageSpeedSlow : storageSpeedFast);
    if(cmd.x && checkLimit(DIR_DOWN, ARM_STORAGE))
    {
        lastStorageDir = DIR_DOWN;
        Storage.set_Speed(teleopStorageSpeed);
    }
    else if(cmd.b && checkLimit(DIR_UP, ARM_STORAGE))
    {
        lastStorageDir = DIR_UP;
        Storage.set_Speed(-teleopStorageSpeed);
    }
    else
    {
        Storage.set_Speed(0.0f);
    }

    // LIMITS OVERRIDE
    if(cmd.back)
    {
        ROS_DEBUG_STREAM((limitOverride ? "Disabling" : "Enabling") << " limit override");
        limitOverride = !limitOverride;
    }
}

void RobotExec::autonomyExec(const robot_msgs::Autonomy& cmd)
{
    ROS_DEBUG_STREAM_COND(this->isDebugMode(), "EXECUTING AUTONOMY CMDS");
    LeftDrive.set_Duty(cmd.leftRatio);
    RightDrive.set_Duty(cmd.rightRatio);

    int autoLiftSpeed = cmd.liftSpeed;
    if(cmd.liftUp && checkLimit(DIR_UP, ARM_LIFT)) {
        Lift.set_Speed(-autoLiftSpeed);
    } else if(cmd.liftDown && checkLimit(DIR_DOWN, ARM_LIFT)) {
        Lift.set_Speed(autoLiftSpeed);
    } else {
        Lift.apply_Brake(liftBrakeCurrent);
    }

    if(cmd.storageUp && checkLimit(DIR_UP, ARM_STORAGE)) {
        Storage.set_Speed(-storageSpeedSlow);
    } else if(cmd.storageDown && checkLimit(DIR_DOWN, ARM_STORAGE)) {
        Storage.set_Speed(storageSpeedSlow);
    } else {
        Storage.set_Speed(0.0f);
    }

    Bucket.set_Duty(cmd.drumRatio);
}

void RobotExec::killMotors()
{
    ROS_DEBUG_STREAM_COND(this->isDebugMode(), "KILL MOTORS");
    LeftDrive.set_Speed(0);
    RightDrive.set_Speed(0);
    Lift.set_Speed(0);
    Lift.apply_Brake(liftBrakeCurrent);
    Storage.set_Speed(0);
    Bucket.set_Speed(0);
}

bool RobotExec::isOnPC()
{
    return onPC;
}

bool RobotExec::isAutonomyActive()
{
    return autonomyActive;
}

void RobotExec::setAutonomyActive(bool active)
{
    autonomyActive = active;
}

bool RobotExec::isDebugMode()
{
    return debug;
}

void RobotExec::setDebugMode(bool active)
{
    debug = active;
}

bool RobotExec::isDead()
{
    return dead;
}

// Robot hibernating due to safety ping disable
void RobotExec::setPingDisabled(bool hiber)
{
    sensors.setStatusLed((hiber ? Sensors::HIBER : (dead ? Sensors::READY : Sensors::ACTIVE)));
}

// Needs to be called frequently, <1s apart
void RobotExec::motorHeartbeat()
{
    LeftDrive.send_Alive();
    RightDrive.send_Alive();
    Lift.send_Alive();
    Storage.send_Alive();
    Bucket.send_Alive();
}

void RobotExec::enforceLimits()
{
    // Check all limits to make sure limits are not
    // passed when holding down button
    if((lastLiftDir == DIR_DOWN && !checkLimit(DIR_DOWN, ARM_LIFT))
    || (lastLiftDir == DIR_UP   && !checkLimit(DIR_UP,   ARM_LIFT)))
        Lift.apply_Brake(liftBrakeCurrent);
    if((lastStorageDir == DIR_DOWN && !checkLimit(DIR_DOWN, ARM_STORAGE))
    || (lastStorageDir == DIR_UP   && !checkLimit(DIR_UP,   ARM_STORAGE)))
        Storage.set_Speed(0.0f);
}

robot_msgs::MotorFeedback RobotExec::getMotorFeedback()
{
    robot_msgs::MotorFeedback fb;

    if(onPC) {
        return fb;
    }

    //get RPM methods will be implemented here
    RxData left_Data = LeftDrive.get_Values();
    RxData right_Data = RightDrive.get_Values();
    RxData lift_Data = Lift.get_Values();
    RxData bucket_Data = Bucket.get_Values();
    RxData storage_Data = Storage.get_Values();

    fb.drumRPM = bucket_Data.rpm;
    fb.drumCurrent = bucket_Data.currentMotor;

    fb.liftDownLimit = !checkLimit(DIR_DOWN, ARM_LIFT, false);
    fb.liftUpLimit = !checkLimit(DIR_UP, ARM_LIFT, false);
    fb.liftPos = sensors.getLiftPosition();
    fb.liftRPM = lift_Data.rpm;
    fb.liftCurrent = lift_Data.currentMotor;

    fb.leftTreadRPM = left_Data.rpm;
    fb.leftTreadCurrent = left_Data.currentMotor;
    fb.rightTreadRPM = right_Data.rpm;
    fb.rightTreadCurrent = right_Data.currentMotor;

    fb.storageDownLimit = sensors.getStorageDownLimit();
    fb.storageUpLimit = sensors.getStorageUpLimit();
    fb.storageCurrent = storage_Data.currentMotor;

    fb.batVoltage = lift_Data.voltageIn;

    fb.rightTreadFault = "";
    fb.leftTreadFault = "";
    fb.drumFault = "";
    fb.liftFault = "";
    fb.storageFault = "";

    return fb;
}

robot_msgs::Status RobotExec::getStatus()
{
    robot_msgs::Status status;

    status.robotCodeActive = true;
    status.autonomyActive  = autonomyActive;
    status.deadmanPressed  = !dead;
    status.limitsOverride  = limitOverride;

    return status;
}

std_msgs::Bool RobotExec::getEnMsg()
{
    return enable;
}

// Returns true if within limits, false if limits hit
// Limits for lift (not storage) can be overridden in case potentiometer fails
bool RobotExec::checkLimit(dir_t dir, arm_t arm, bool printlimit)
{
    bool ret = true;
    float liftAngle = sensors.getLiftPosition();
    if(arm == ARM_LIFT && !limitOverride)
    {
        // Going down increases angle
        if(dir == DIR_DOWN && liftAngle > liftDownLimit)
            ret = false;
        else if(dir == DIR_UP && liftAngle < liftUpLimit)
            ret = false;
    }
    else if(arm == ARM_STORAGE)
    {
        // Storage uses limit switches as well as lift angle
        // Storage should not move in any direction if lift is up
        if(liftAngle < storageLiftLimit && !limitOverride)
            ret = false;
        else if(dir == DIR_DOWN && sensors.getStorageDownLimit())
            ret = false;
        else if(dir == DIR_UP && sensors.getStorageUpLimit())
            ret = false;
    }

    if(!ret && printlimit)
    {
        const char* dirStr = (dir == DIR_DOWN ? "down" : "up");
        const char* armStr = (arm == ARM_LIFT ? "lift" : "storage");
        ROS_DEBUG_STREAM("Hit " << armStr << " " << dirStr << " limit!");
    }
    return ret;
}
