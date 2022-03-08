package org.firstinspires.ftc.teamcode.b_hardware.betterSubsystems;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake extends SubsystemBase {
    public static double intakeSpeed = 0.8;
    private MotorEx intake;
    public ColorSensor bucketSensor;

    private Servo flip;



    private static boolean bucketFlipped = false;




    public Intake(OpMode opMode) {

        //TODO: reverse if not right
        intake = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_312);
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intake.motor.setDirection(DcMotorSimple.Direction.REVERSE);


        flip = opMode.hardwareMap.servo.get("flip");
        flip.setDirection(Servo.Direction.FORWARD);


        bucketSensor = opMode.hardwareMap.get(ColorSensor.class, "bucketSensor");


    }


//    public double alphaValue() { return bucketSensor.alpha();}
//
//    public boolean isFreightInClaw() {
//        return clawSensor.alpha() > 500;
//    }
//


    public boolean isFreightInBucket() {
        return bucketSensor.alpha() > 500;
    }



    public void run(double speed){
        intake.set(speed);
    }


    public void run(){
        run(intakeSpeed);
    }



    public void spit(){
        run(-intakeSpeed);
    }


    public void stop(){
        intake.stopMotor();
    }

    public void flipBucket() {
        flip.setPosition(0.6); //0.15
        bucketFlipped = true;
    }

    public void unflipBucket() {
        flip.setPosition(0.1); //0.55
        bucketFlipped = false;
    }

    public void toggleBucket() {
        if (bucketFlipped) {
            unflipBucket();
        } else if (!bucketFlipped) {
            flipBucket();
        }
    }



  // @Override
//    public void periodic() {
//        if (isFreightIn() && !bucketFlipped) {
//            flipBucket();
//        } else if (isFreightIn() && bucketFlipped) {
//            run();
//            if (!isFreightIn() && bucketFlipped){
//                stop();
//                unflipBucket();
//            }
//        }
//    }
}
