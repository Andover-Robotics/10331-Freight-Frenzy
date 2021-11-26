package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

public class Outake extends SubsystemBase {
    private static final double OPEN = 0.0;
    private static final double CLOSE = 0.5;
    private Servo gateFlap;


    private enum state {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public state runState = state.OFF;


    public Gate(OpMode opMode) {
        gateFlap = opMode.hardwareMap.servo.get("gateFlap");
        gateFlap.setDirection(Servo.Direction.REVERSE);
        gatetFlap.setPosition(OPEN);
    }

    public void gateToggle() {
        if (runState == state.OFF) {
            closeGateFlap();
        } else {
            openGateFlap();
        }
    }


    public void closeGateFlap() {
        gateFlap.setDirection(Direction.FOWARD);
        gateFlap.setPosition(CLOSE);
    }


    public void openGateFlap() {
        gateFlap.setDirection(Direction.REVERSE);
        gateFlap.setPosition(OPEN);
    }

}