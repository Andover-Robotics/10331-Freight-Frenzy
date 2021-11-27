package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate extends SubsystemBase {
    private static final double OPEN = 0.0;
    private static final double CLOSE = 0.5;
    private Servo gate;


    public enum State {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public State runState = State.OFF;


    public Gate(OpMode opMode) {
        gate = opMode.hardwareMap.servo.get("gate");
        gate.setDirection(Servo.Direction.REVERSE);
        gate.setPosition(OPEN);
    }

    public void gateToggle() {
        if (runState == State.OFF) {
            closeGateFlap();
        } else {
            openGateFlap();
        }
    }


    public void closeGateFlap() {
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(CLOSE);
    }


    public void openGateFlap() {
        gate.setDirection(Servo.Direction.REVERSE);
        gate.setPosition(OPEN);
    }

}