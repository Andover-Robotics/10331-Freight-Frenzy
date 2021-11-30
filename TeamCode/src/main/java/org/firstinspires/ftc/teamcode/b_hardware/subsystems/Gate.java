package org.firstinspires.ftc.teamcode.b_hardware.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate extends SubsystemBase {
    private Servo leftGate;
    private Servo rightGate;




    public Gate(OpMode opMode) {
        leftGate = opMode.hardwareMap.servo.get("leftGate");
        leftGate.setPosition(0.75);
        leftGate.setDirection(Servo.Direction.REVERSE);

        rightGate = opMode.hardwareMap.servo.get("rightGate");
        rightGate.setPosition(0.25);
        rightGate.setDirection(Servo.Direction.FORWARD);
    }



    public void closeLeft() {
        leftGate.setPosition(0.75);
    }


    public void openLeft() {
        leftGate.setPosition(0.44);
    }


    public void closeRight() {
        rightGate.setPosition(0.25);
    }


    public void openRight() {
        rightGate.setPosition(0.55);
    }

    public void takeInRight() {
        rightGate.setPosition(0.3);
    }

    public void takeInLeft() {
        leftGate.setPosition(0.7);
    }


}