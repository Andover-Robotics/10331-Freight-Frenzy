package org.firstinspires.ftc.teamcode.b_hardware.betterSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {
    private Servo left;
    private Servo rightGate;
    private MotorEx motor;


    //TODO: find positions
    private static double
            leftOpen = 0.2,
            leftClosed = 0.1,
            rightOpen = 0.15,
            rightClosed = 0.25;


    public Outtake(OpMode opMode){

        left = opMode.hardwareMap.servo.get("leftClaw");
        left.setDirection(Servo.Direction.FORWARD);


        rightGate = opMode.hardwareMap.servo.get("rightGate");
        rightGate.setDirection(Servo.Direction.FORWARD);

        clamp();

        motor = new MotorEx(opMode.hardwareMap, "outtake", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPositionCoefficient(0.15);

    }


    public void clamp(){
        left.setPosition(leftClosed);
        rightGate.setPosition(rightClosed);
    }

    public void open(){

        left.setPosition(leftOpen);
        rightGate.setPosition(rightOpen);
    }

    public void capping(){
        left.setPosition(0.25);
        rightGate.setPosition(0.15);
    }


    //5000
    //3000
    //2783
    public void runToHigh(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(5);
        motor.setTargetPosition(1205);//5000//1454
        while(!motor.atTargetPosition()){
          motor.set(0.8);
        }
        motor.stopMotor();
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void runToMid(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(5);
        motor.setTargetPosition(829);//3000//1024
        while(!motor.atTargetPosition()){
            motor.set(0.8);
        }
        motor.stopMotor();
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void runToLow(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(5);
        motor.setTargetPosition(600);//2783//5591
        while(!motor.atTargetPosition()){
            motor.set(0.8);
        }
        motor.stopMotor();
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void restArm(){
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(0);
        while(!motor.atTargetPosition()){
            motor.set(0.8);
        }
        motor.stopMotor();
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void runArmSlow() {
        motor.set(0.5);
    }

    public void downArmSlow(){
        motor.set(-1);
    }

       public void runArm(double power){
        motor.set(power*2.0);
    }

    public void stopArm(){
        motor.stopMotor();
    }

//    @Override
//    public void periodic() {
//        if (motor.getCurrentPosition() < 0.0) {
//            open();
//        }
//    }
}

