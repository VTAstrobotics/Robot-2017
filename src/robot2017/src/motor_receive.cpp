#include "motor_receive.h"
#include <vesc_bbb_uart/bldc.h>

MotorsReceive::MotorsReceive(RobotExec& exec) : exec(exec) {
}

void MotorsReceive::update() {
    switch(read_mode){
        case RequestMotor1:
            exec.LeftDrive.request_Values();
            start_timer(timerid, TIMER_SP);
            read_mode = ReadMotor1;
            break;
        case ReadMotor1:
            if (check_timer(timerid)){
                exec.LeftDrive.read_Data();
                // Return values for use by application
                //testData = exec.LeftDrive.get_Values();
                exec.LeftDrive.print_Data();
                read_mode = RequestMotor2;
            }
            break;

        case RequestMotor2:
            exec.RightDrive.request_Values();
            start_timer(timerid, TIMER_SP);
            read_mode = ReadMotor2;
            break;
        case ReadMotor2:
            if (check_timer(timerid)){
                exec.RightDrive.read_Data();
                // Return values for use by application
                //testData = exec.RightDrive.get_Values();
                exec.RightDrive.print_Data();
                read_mode = RequestMotor3;
            }
            break;

        case RequestMotor3:
            exec.Lift.request_Values();
            start_timer(timerid, TIMER_SP);
            read_mode = ReadMotor3;
            break;
        case ReadMotor3:
            if (check_timer(timerid)){
                exec.Lift.read_Data();
                // Return values for use by application
                //testData = exec.Lift.get_Values();
                exec.Lift.print_Data();
                read_mode = RequestMotor4;
            }
            break;

        case RequestMotor4:
            exec.Storage.request_Values();
            start_timer(timerid, TIMER_SP);
            read_mode = ReadMotor4;
            break;
        case ReadMotor4:
            if (check_timer(timerid)){
                exec.Storage.read_Data();
                // Return values for use by application
                //testData = exec.Storage.get_Values();
                exec.Storage.print_Data();
                read_mode = RequestMotor5;
            }
            break;

        case RequestMotor5:
            exec.Bucket.request_Values();
            start_timer(timerid, TIMER_SP);
            read_mode = ReadMotor5;
            break;
        case ReadMotor5:
            if (check_timer(timerid)){
                exec.Bucket.read_Data();
                // Return values for use by application
                //testData = exec.Bucket.get_Values();
                exec.Bucket.print_Data();
                read_mode = RequestMotor1; // make state-machine circular
            }
            break;
            
        default:
            break;
    }
}
