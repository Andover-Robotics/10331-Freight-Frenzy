package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate extends SubsystemBase {
    private static final double OPENCLOSE = 0.25;
    private static final double TAKEIN = 0.05;
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
        leftGate.setPosition(0.0);
        leftGate.setDirection(Servo.Direction.FORWARD);

        rightGate = opMode.hardwareMap.servo.get("rightGate");
        rightGate.setPosition(0.0);
        rightGate.setDirection(Servo.Direction.REVERSE);
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
//        leftGate.setDirection(Servo.Direction.REVERSE);
        leftGate.setPosition(OPENCLOSE);
        runState = State.OFF;
    }


    public void openLeftGateFlap() {
//        leftGate.setDirection(Servo.Direction.FORWARD);
        leftGate.setPosition(OPENCLOSE);
        runState = State.LEFT;

    }


    public void closeRightGateFlap() {
//        rightGate.setDirection(Servo.Direction.FORWARD);
        rightGate.setPosition(OPENCLOSE);
        runState = State.OFF;
    }


    public void openRightGateFlap() {
//        rightGate.setDirection(Servo.Direction.REVERSE);
        rightGate.setPosition(OPENCLOSE);
        runState = State.RIGHT;
    }

    public void takeInRightGateFlap() {
//        rightGate.setDirection(Servo.Direction.REVERSE);
        rightGate.setPosition(TAKEIN);
        runState = State.LEFT;

    }

    public void takeInLeftGateFlap() {
//        leftGate.setDirection(Servo.Direction.FORWARD);
        leftGate.setPosition(-TAKEIN);
        runState = State.RIGHT;

    }

    public void closeTakeInLeft(){
//        leftGate.setDirection(Servo.Direction.REVERSE);
        leftGate.setPosition(TAKEIN);
        runState = State.LEFT;
    }


    public void closeTakeInRight(){
        rightGate.setDirection(Servo.Direction.FORWARD);
        rightGate.setPosition(TAKEIN);
        runState = State.RIGHT;
    }





}