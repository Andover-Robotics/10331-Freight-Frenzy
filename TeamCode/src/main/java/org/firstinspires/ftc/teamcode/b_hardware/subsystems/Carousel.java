package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Carousel {
    public static final double SPEED = 0.25;
    private MotorEx carousel;

    public Carousel (OpMode opMode) {
        carousel = new MotorEx(opMode.hardwareMap, "carousel", Motor.GoBILDA.RPM_312);
        carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        carousel.set(0);
    }

    public void run (){
        carousel.set(SPEED);

    }

    public void reverse (){
       carousel.set(-SPEED);
    }


    public void stop(){
        carousel.set(0.0);
    }

}

