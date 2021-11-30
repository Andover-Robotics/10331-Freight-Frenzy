package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate extends SubsystemBase {
    private Servo leftClaw;
    private Servo rightClaw;



    public Gate(OpMode opMode) {
        leftClaw = opMode.hardwareMap.servo.get("leftClaw");
        leftClaw.setPosition(0.75);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        rightClaw = opMode.hardwareMap.servo.get("rightGate");
        rightClaw.setPosition(0.25);
        rightClaw.setDirection(Servo.Direction.FORWARD);
    }

    public void closeLeft() {
        leftClaw.setPosition(0.75);
    }


    public void openLeft() {
        leftClaw.setPosition(0.44);
    }


    public void closeRight() {
        rightClaw.setPosition(0.25);
    }


    public void openRight() {
        rightClaw.setPosition(0.55);
    }

    public void takeInRight() {
        rightClaw.setPosition(0.3);
    }

    public void takeInLeft() {
        leftClaw.setPosition(0.7);
    }


}