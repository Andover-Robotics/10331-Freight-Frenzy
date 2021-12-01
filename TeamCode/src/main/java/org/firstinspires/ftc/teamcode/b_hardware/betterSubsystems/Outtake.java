package org.firstinspires.ftc.teamcode.b_hardware.betterSubsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class Outtake {
    private Servo leftGate;
    private Servo rightGate;
    private MotorEx motor;

    //TODO: find positions
    private static double leftOpen = 0.44,
    leftClosed = 0.75,
    leftReceive = 0.7,
    rightOpen = 0.55,
    rightClosed = 0.25,
    rightReceive = 0.3;

    public Outtake(OpMode opMode){
        leftGate = opMode.hardwareMap.servo.get("leftClaw");
        leftGate.setDirection(Servo.Direction.REVERSE);

        rightGate = opMode.hardwareMap.servo.get("rightGate");
        rightGate.setDirection(Servo.Direction.FORWARD);

        clamp();

        motor = new MotorEx(opMode.hardwareMap, "outtake", Motor.GoBILDA.RPM_312);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void receive(){//TODO: test if the alliance is right
        if(GlobalConfig.alliance == GlobalConfig.Alliance.RED){
            receiveLeft();
        }else{
            receiveRight();
        }
    }

    public void receiveRight(){
        leftGate.setPosition(leftReceive);
        rightGate.setPosition(rightOpen);
    }

    public void receiveLeft(){
        leftGate.setPosition(leftOpen);
        rightGate.setPosition(rightReceive);
    }

    public void clamp(){
        leftGate.setPosition(leftClosed);
        rightGate.setPosition(rightClosed);
    }

    public void open(){
        leftGate.setPosition(leftOpen);
        rightGate.setPosition(rightOpen);
    }

    public void runArm(double power){
        motor.set(power);
    }

    public void stopArm(){
        motor.stopMotor();
    }
}
