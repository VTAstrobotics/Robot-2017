//teleop_exec.h
//handles teleop calculations
//calls appropriate motor functions

#include "teleop_msg/Teleop.h"
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
        void teleopExec(const teleop_msg::Teleop& cmd);
        void killMotors();
};

#endif
