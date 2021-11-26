package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Outtake extends SubsystemBase {
    public static final double SPEED = 0.25;
    private MotorEx armMotor;

    public enum State {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public State runState = State.OFF;


    public Outtake (OpMode opMode){
        armMotor = new MotorEx(opMode.hardwareMap, "armMotor", Motor.GoBILDA.RPM_312);
        armMotor.setRunMode(Motor.RunMode.RawPower);
        armMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

//    public void gateToggle (){
//        if(runState == State.OFF) {
//            run();
//        }
//        else {
//            stop();
//        }
//    }


    public void run(){
        armMotor.set(SPEED);
        runState = State.ON;
    }


    public void down(){
        armMotor.set(-SPEED);
        runState = State.ON;
    }


    public void stop(){
        armMotor.set(0.0);
        runState = State.OFF;
    }


}