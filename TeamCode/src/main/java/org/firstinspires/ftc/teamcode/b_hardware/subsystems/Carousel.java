package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GlobalConfig;

@Config
public class Carousel extends SubsystemBase {
    public static final double SPEED = 0.25;
    private MotorEx motor;

    public Carousel(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "carousel", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.motor.setDirection(GlobalConfig.alliance == GlobalConfig.Alliance.RED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void run(){
        motor.set(SPEED);
    }

    public void stop(){
        motor.set(0);
    }
}
