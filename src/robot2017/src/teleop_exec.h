//teleop_exec.h
//handles teleop calculations
//calls appropriate motor functions

#include "robot_msgs/Teleop.h"
#include "motors.h"

#ifndef TELEOP_EXEC_H
#define TELEOP_EXEC_H

class TeleopExec
{
    private:
        bool dead;
        float leftRatio;
        float rightRatio;

    public:
        void teleopExec(const robot_msgs::Teleop& cmd);
        void killMotors();
};

#endif
