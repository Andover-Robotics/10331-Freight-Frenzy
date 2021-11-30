package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends SubsystemBase {
    public static final double SPEED=-0.5;
    MotorEx leftIntake;
    MotorEx rightIntake;

    public enum STATE {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public STATE runState = STATE.OFF;

    public Intake(OpMode opMode){
        leftIntake = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_312);
        leftIntake.setRunMode(Motor.RunMode.RawPower);
        leftIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightIntake = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_312);
        rightIntake.setRunMode(Motor.RunMode.RawPower);
        rightIntake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
//

//
//    public void runToggle() {
//        if(runState == STATE.OFF) {
//            run();
//        }
//        else {
//            stop();
//        }
//    }
//
//    public void run(){
//        leftIntake.set(SPEED);
//        rightIntake.set(SPEED);
//        runState = STATE.ON;
//    }

//    public void reverse() {
//        runState = state.REVERSE;
//        leftIntake.set(-SPEED);
//        rightIntake.set(-SPEED);
//    }
//

//    public void reverseLeft() {
//        leftIntake.set(SPEED);
//    }
//
//    public void reverseRight() {
//        rightIntake.set(-SPEED);
//    }

    public void runLeft(){
        leftIntake.set(SPEED);
        rightIntake.set(0.0);
        runState = STATE.LEFT;
    }

    public void runRight(){
        leftIntake.set(0.0);
        rightIntake.set(SPEED);
        runState = STATE.RIGHT;
    }

//    public void switchIntake(){
//        if (runState == STATE.LEFT) {
//            runRight();
//        }
//        else {
//            runLeft();
//        }
//    }

    public void stop(){
        leftIntake.set(0.0);
        rightIntake.set(0.0);
        runState = STATE.OFF;
    }
}