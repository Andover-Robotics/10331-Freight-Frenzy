package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate extends SubsystemBase {
    private static final double OPEN = 0.0;
    private static final double CLOSE = 0.5;
    private Servo gateFlap;


    public enum State {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public State runState = State.OFF;


    public Gate(OpMode opMode) {
        gateFlap = opMode.hardwareMap.servo.get("gateFlap");
        gateFlap.setDirection(Servo.Direction.REVERSE);
        gateFlap.setPosition(OPEN);
    }

    public void gateToggle() {
        if (runState == State.OFF) {
            closeGateFlap();
        } else {
            openGateFlap();
        }
    }


    public void closeGateFlap() {
        gateFlap.setDirection(Servo.Direction.FORWARD);
        gateFlap.setPosition(CLOSE);
    }


    public void openGateFlap() {
        gateFlap.setDirection(Servo.Direction.REVERSE);
        gateFlap.setPosition(OPEN);
    }

}