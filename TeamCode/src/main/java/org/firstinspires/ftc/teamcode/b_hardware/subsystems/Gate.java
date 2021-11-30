package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate extends SubsystemBase {
    private static final double OPENCLOSE = 0.25;
    private static final double TAKEIN = 0.05;
    private Servo leftGate;
    private Servo rightGate;




    public Gate(OpMode opMode) {
        leftGate = opMode.hardwareMap.servo.get("leftGate");
        leftGate.setPosition(0.0);
        leftGate.setDirection(Servo.Direction.FORWARD);

        rightGate = opMode.hardwareMap.servo.get("rightGate");
        rightGate.setPosition(0.0);
        rightGate.setDirection(Servo.Direction.REVERSE);
    }



    public void closeLeft() {
        leftGate.setPosition(0.0);
    }


    public void openLeft() {
        leftGate.setPosition(OPENCLOSE);
    }


    public void closeRight() {
        rightGate.setPosition(0.0);
    }


    public void openRight() {
        rightGate.setPosition(OPENCLOSE);
    }

    public void takeInRight() {
        rightGate.setPosition(TAKEIN);
    }

    public void takeInLeft() {
        leftGate.setPosition(TAKEIN);
    }

    public void closeTakeInLeft(){
        leftGate.setPosition(0.0);
    }


    public void closeTakeInRight(){
        rightGate.setPosition(0.0);
    }

}