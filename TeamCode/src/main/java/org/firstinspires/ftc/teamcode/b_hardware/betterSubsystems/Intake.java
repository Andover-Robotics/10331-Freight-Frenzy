package org.firstinspires.ftc.teamcode.b_hardware.betterSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends SubsystemBase {
    public static double intakeSpeed = 0.7;
    private MotorEx motorLeft;
//    private MotorEx motorRight;

    public Intake(OpMode opMode){

        //TODO: reverse if not right
        motorLeft = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_435);
        motorLeft.setRunMode(Motor.RunMode.RawPower);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorLeft.motor.setDirection(DcMotorSimple.Direction.REVERSE);

//        motorRight = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_435);
//        motorRight.setRunMode(Motor.RunMode.RawPower);
//        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        motorRight.motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

//    public void run(double speed){                         game
//        motorLeft.setVelocity(speed);
//        motorRight.setVelocity(speed);
//    }

    public void runLeft(double speed){
        motorLeft.set(speed);
    }

//    public void runRight(double speed){
//        motorRight.set(speed);
//    }

    public void runLeft(){
        runLeft(intakeSpeed);
    }

//    public void runRight(){
//        runRight(intakeSpeed);
//    }

    public void spitLeft(){
        runLeft(-intakeSpeed);
    }

//    public void spitRight(){
//        runRight(-intakeSpeed);
//    }

//    public void run(){
//        run(intakeSpeed);
//    }

//    public void spit(){
//        run(-0.5);
//    }

    public void stop(){
        motorLeft.stopMotor();
//        motorRight.stopMotor();
    }
}
