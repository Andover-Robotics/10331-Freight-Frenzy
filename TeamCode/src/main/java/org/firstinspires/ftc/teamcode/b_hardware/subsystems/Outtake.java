package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

public class Outake extends SubsystemBase {
    public static final double SPEED = 0.25;
    public static final double REVERSE = -0.25;
    private MotorEx armMotor;

    private enum state {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public state runState = state.OFF;


    public Outtake(OpMode opMode){
        armMotor = new MotorEx(opMode.hardwareMap, "armMotor", Motor.GoBILDA.RPM_312);
        armMotor.setRunMode(Motor.RunMode.RawPower);
        armMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void gateToggle (){
        if(runState == state.OFF) {
            run();
        }
        else {
            stop();
        }
    }


    public void run(){
        armMotor.set(SPEED);
        runState = STATE.ON;
    }


    public void down(){
        armMotor.set(REVERSE);
        runState = STATE.ON;
    }


    public void stop(){
        armMotor.set(0.0);
        runState = state.OFF;
    }


}