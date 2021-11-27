package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Outtake extends SubsystemBase {
    public static final double SPEED = 0.25;
    private MotorEx outtake;

    public enum State {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public State runState = State.OFF;


    public Outtake (OpMode opMode){
        outtake = new MotorEx(opMode.hardwareMap, "outtake", Motor.GoBILDA.RPM_312);
        outtake.setRunMode(Motor.RunMode.RawPower);
        outtake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        outtake.set(SPEED*4);
        runState = State.ON;
    }



    public void down(){
        outtake.set(-SPEED);
        runState = State.ON;
    }


    public void stop(){
        outtake.set(0.0);
        runState = State.OFF;
    }


}