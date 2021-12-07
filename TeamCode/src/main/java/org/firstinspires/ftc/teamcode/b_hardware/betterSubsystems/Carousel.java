package org.firstinspires.ftc.teamcode.b_hardware.betterSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Carousel extends SubsystemBase {
    public static double carouselSpeed = 0.25;
    private MotorEx motor;

    public Carousel(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, " carousel", Motor.GoBILDA.RPM_435);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
       // motor.motor.setDirection(GlobalConfig.alliance == GlobalConfig.Alliance.RED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void run(){
        motor.set(carouselSpeed);
    }

    public void reverse() {
        motor.set(-carouselSpeed);
    }

    public void stop(){
        motor.stopMotor();
    }

}
