#include "robot_exec.h"
#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_msgs/MotorFeedback.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>

const char* motorPath = "/dev/ttyO1";  // Connected via UART
// const char* motorPath = "/dev/ttyUSB0" // Connected via USB

const float liftSpeed    = 4000; // RPM
const float storageSpeed = 8000; // RPM

RobotExec::RobotExec(bool onPC, bool debug, bool autoActive)
    : dead(true), onPC(onPC), debug(debug), autonomyActive(autoActive),
      leftRatio(0.0f), rightRatio(0.0f), sensors(onPC), prevState(false),
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

    // Make sure autonomy doesn't run if kill switch is pressed
    if(this->autonomyActive && !dead)
    {
        autonomyExec(cmd);
    }
    publishMotors();
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
// ==== MOTOR CONTROLS ====
// Driving:  duty cycle (% voltage, -1.0 to 1.0)
// Drum:     duty cycle (% voltage, -1.0 to 1.0)
// Lift:     speed (raw RPM)
// Storage:  speed (raw RPM)
void RobotExec::teleopExec(const robot_msgs::Teleop& cmd)
{
    // DEADMAN + physical kill switch
    bool shouldKill = dead || !cmd.lb;
    ROS_DEBUG_STREAM("Robot killed: " << shouldKill);
    if(shouldKill)
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
    if(cmd.y) {
        Lift.set_Speed(-liftSpeed);
    }
    else if(cmd.a)
    {
        Lift.set_Speed(liftSpeed);
    }
    else
    {
        Lift.set_Pos(0.0f);
    }

    // SECONDARY STORAGE
    if(cmd.b)
    {
        Storage.set_Speed(storageSpeed);
    }
    else if(cmd.x)
    {
        Storage.set_Speed(-storageSpeed);
    }
    else
    {
        Storage.set_Speed(0.0f);
    }
}

void RobotExec::autonomyExec(const robot_msgs::Autonomy& cmd)
{
    ROS_DEBUG_STREAM_COND(this->isDebugMode(), "EXECUTING AUTONOMY CMDS");
    LeftDrive.set_Duty(cmd.leftRatio);
    RightDrive.set_Duty(cmd.rightRatio);

    if(cmd.liftUp) {
        Lift.set_Speed(-liftSpeed);
    } else if(cmd.liftDown) {
        Lift.set_Speed(liftSpeed);
    } else {
        Lift.set_Pos(0.0f);
    }

    if(cmd.storageUp) {
        Storage.set_Speed(storageSpeed);
    } else if(cmd.storageDown) {
        Storage.set_Speed(-storageSpeed);
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

// Needs to be called frequently, <1s apart
void RobotExec::motorHeartbeat()
{
    LeftDrive.send_Alive();
    RightDrive.send_Alive();
    Lift.send_Alive();
    Storage.send_Alive();
    Bucket.send_Alive();
}

// Call frequently to check if kill button on robot is pressed
// If pressed, will kill motors and disable teleop until released
void RobotExec::checkKillButton() {
    dead = sensors.getKillButton();
    if(dead)
    {
        killMotors();
    }
}

robot_msgs::MotorFeedback RobotExec::publishMotors()
{
    robot_msgs::MotorFeedback fb;

    //get RPM methods will be implemented here
    RxData left_Data = LeftDrive.get_Values();
    RxData right_Data = RightDrive.get_Values();
    RxData lift_Data = Lift.get_Values();
    RxData bucket_Data = Bucket.get_Values();

    ROS_DEBUG_STREAM_COND(this->isDebugMode(), "GETTING MOTOR DATA");

    fb.drumRPM = bucket_Data.rpm;

    fb.liftPos = sensors.getLiftPosition();
    fb.liftRPM = lift_Data.rpm;

    fb.leftTreadRPM = left_Data.rpm;
    fb.rightTreadRPM = right_Data.rpm;

    fb.liftCurrent = lift_Data.currentMotor;
    fb.drumCurrent = bucket_Data.currentMotor;

    // TODO items not yet implemented
    fb.leftStorageWeight = sensors.getLeftStorageWeight();
    fb.rightStorageWeight = sensors.getRightStorageWeight();
    fb.storagePos = sensors.getStoragePosition();
    
    fb.rightTreadFault = "";
    fb.leftTreadFault = "";
    fb.drumFault = "";
    fb.liftFault = "";
    fb.storageFault = "";

    return fb;
}

std_msgs::Bool getEnMsg()
{
    return enable;
}
