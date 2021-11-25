package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends SubsystemBase {
    public static final double SPEED=-0.5;
    MotorEx leftIntake;

    private enum state {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public state runState = state.OFF;

    public Intake(OpMode opMode){
        leftIntake = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_312);
        leftIntake.setRunMode(Motor.RunMode.RawPower);
        leftIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightIntake = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_312);
//        rightIntake.setRunMode(Motor.RunMode.RawPower);
//        rightIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
//
    public void runToggle() {
        if(runState == state.OFF) {
            run();
        }
        else {
            stop();
        }
    }

    public void run(){
        leftIntake.set(SPEED);
//        rightIntake.set(SPEED);
        runState = state.ON;
    }

//    public void reverse() {
//        runState = state.REVERSE;
//        leftIntake.set(-SPEED);
////        rightIntake.set(-SPEED);
//    }
//
//    public void runLeft(){
////        rightIntake.set(0.0);
//        leftIntake.set(SPEED);
//        runState = state.LEFT;
//    }

//    public void runRight(){
//        leftIntake.set(0.0);
////        rightIntake.set(SPEED);
//        runState = state.RIGHT;
//    }

//    public void switchIntake(){
//        if (runState == state.LEFT) {
//            runRight();
//        }
//        else {
//            runLeft();
//        }
//    }

    public void stop(){
        leftIntake.set(0.0);
//        rightIntake.set(0.0);
        runState = state.OFF;
    }
}

}