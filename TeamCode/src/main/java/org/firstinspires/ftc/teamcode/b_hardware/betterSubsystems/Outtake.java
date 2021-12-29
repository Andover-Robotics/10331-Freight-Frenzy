package org.firstinspires.ftc.teamcode.b_hardware.betterSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.teamcode.GlobalConfig;

public class Outtake extends SubsystemBase {
    private Servo left;
    private Servo rightGate;
    private MotorEx motor;

    //TODO: find positions
    private static double leftOpen = 0.6,
    leftClosed = 0.9,
    leftReceive = 0.85,
    leftNormal = 0.3,
    rightOpen = 0.25,
    rightClosed = 0.1,
    rightReceive = 0.2;

    public Outtake(OpMode opMode){
        left = opMode.hardwareMap.servo.get("leftClaw");
        left.setDirection(Servo.Direction.FORWARD);

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
        left.setPosition(leftNormal);
        rightGate.setPosition(rightClosed);
    }

    public void open(){

        left.setPosition(leftNormal);
        rightGate.setPosition(rightOpen);
    }
    //5000
    //3000
    //2783
    public void runToHigh(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(40);
        motor.setTargetPosition(5000);
        while(!motor.atTargetPosition()){
          motor.set(0.5);
        }
        motor.stopMotor();
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void runToMid(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(40);
        motor.setTargetPosition(3000);
        while(!motor.atTargetPosition()){
            motor.set(0.5);
        }
        motor.stopMotor();
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void runToLow(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(40);
        motor.setTargetPosition(2783);
        while(!motor.atTargetPosition()){
            motor.set(0.5);
        }
        motor.stopMotor();
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void runArm(double power){
        motor.set(power*0.5);
    }

    public void stopArm(){
        motor.stopMotor();
    }
}
