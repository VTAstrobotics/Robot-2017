//teleop_exec.h
//handles teleop calculations
//calls appropriate motor functions

#include "robot2017/Teleop.h"
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
        void teleopExec(const robot2017::Teleop& cmd);
        void killMotors();
};

#endif