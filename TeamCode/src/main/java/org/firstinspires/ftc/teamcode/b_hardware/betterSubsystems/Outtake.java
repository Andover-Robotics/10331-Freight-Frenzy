package org.firstinspires.ftc.teamcode.b_hardware.betterSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class Outtake extends SubsystemBase {
    private Servo left;
    private Servo rightGate;
    private MotorEx motor;

    //TODO: find positions
    private static double leftOpen = 0.6,
    leftClosed = 0.9,
    leftReceive = 0.85,
    rightOpen = 0.4,
    rightClosed = 0.15,
    rightReceive = 0.2;

    public Outtake(OpMode opMode){
        left = opMode.hardwareMap.servo.get("leftClaw");
        left.setDirection(Servo.Direction.REVERSE);

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
        left.setPosition(leftReceive);
        rightGate.setPosition(rightOpen);
    }

    public void receiveLeft(){
        left.setPosition(leftOpen);
        rightGate.setPosition(rightReceive);
    }

    public void clamp(){
        left.setPosition(leftClosed);
        rightGate.setPosition(rightClosed);
    }

    public void open(){
        left.setPosition(leftOpen);
        rightGate.setPosition(rightOpen);
    }

    public void runArm(double power){
        motor.set(power*0.5);
    }

    public void stopArm(){
        motor.stopMotor();
    }
}
