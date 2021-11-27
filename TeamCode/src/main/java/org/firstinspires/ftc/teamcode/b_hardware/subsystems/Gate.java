package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate extends SubsystemBase {
    private static final double OPEN = 0.0;
    private static final double CLOSE = 0.5;
    private Servo leftGate;
    private Servo rightGate;


    public enum State {
        ON,
        OFF,
        REVERSE,
        LEFT,
        RIGHT
    }

    public State runState = State.OFF;


    public Gate(OpMode opMode) {
        leftGate = opMode.hardwareMap.servo.get("leftGate");
        leftGate.setPosition(OPEN);

        rightGate = opMode.hardwareMap.servo.get("rightGate");
        rightGate.setPosition(OPEN);
    }

//    public void leftGateToggle() {
//        if (runState == State.OFF) {
//            closeLeftGateFlap();
//        } else {
//            openLeftGateFlap();
//        }
//    }
//
//    public void rightGateToggle() {
//        if (runState == State.OFF) {
//            closeRightGateFlap();
//        } else {
//            openRightGateFlap();
//        }
//    }


    public void closeLeftGateFlap() {
        leftGate.setDirection(Servo.Direction.REVERSE);
        leftGate.setPosition(CLOSE);
        runState = State.OFF;
    }


    public void openLeftGateFlap() {
        leftGate.setDirection(Servo.Direction.FORWARD);
        leftGate.setPosition(OPEN);
        runState = State.LEFT;

    }


    public void closeRightGateFlap() {
        rightGate.setDirection(Servo.Direction.REVERSE);
        rightGate.setPosition(CLOSE);
        runState = State.OFF;
    }


    public void openRightGateFlap() {
        rightGate.setDirection(Servo.Direction.FORWARD);
        rightGate.setPosition(OPEN);
        runState = State.RIGHT;
    }



}