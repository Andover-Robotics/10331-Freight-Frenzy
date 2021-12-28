package org.firstinspires.ftc.teamcode.b_hardware.betterSubsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drivetrain {
    private MotorEx motorFL;
    private MotorEx motorFR;
    private MotorEx motorBL;
    private MotorEx motorBR;

    public Drivetrain(OpMode opMode){
        motorFL = new MotorEx(opMode.hardwareMap, "motorFL", Motor.GoBILDA.RPM_312);
        motorFL.setRunMode(Motor.RunMode.PositionControl);
        motorFL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
       // motorFL.motor.setDirection();

        motorFR = new MotorEx(opMode.hardwareMap, "motorFR", Motor.GoBILDA.RPM_312);
        motorFR.setRunMode(Motor.RunMode.PositionControl);
        motorFR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorBL = new MotorEx(opMode.hardwareMap, "motorBL", Motor.GoBILDA.RPM_312);
        motorBL.setRunMode(Motor.RunMode.PositionControl);
        motorBL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorBR = new MotorEx(opMode.hardwareMap, "motorBR", Motor.GoBILDA.RPM_312);
        motorBR.setRunMode(Motor.RunMode.PositionControl);
        motorBR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);



    }
    
    public static void goToTicks(double ticks){
//        motorFL.setTargetPosition(ticks);
//        motorFL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        motorFR.motor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        motorBL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        motorBR.motor.setDirection(DcMotorSimple.Direction.REVERSE);


    }




}
