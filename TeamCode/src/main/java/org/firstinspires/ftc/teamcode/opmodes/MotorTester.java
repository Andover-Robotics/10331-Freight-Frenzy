package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GlobalConfig;

import java.util.List;

@TeleOp(name="motor tester", group="testing")
public class MotorTester extends BaseOpMode {
    MotorEx motorFL, motorFR, motorBL, motorBR;

    @Override
    public void subInit() {
        motorBL = new MotorEx(hardwareMap, GlobalConfig.motorBL);
        motorFR = new MotorEx(hardwareMap, GlobalConfig.motorFR);
        motorFL = new MotorEx(hardwareMap, GlobalConfig.motorFL);
        motorBR = new MotorEx(hardwareMap, GlobalConfig.motorBR);

    }

    @Override
    public void subLoop() {
        if(buttonSignal(GamepadKeys.Button.A)) {
            motorBL.set(1.0);
        } else if(buttonSignal(GamepadKeys.Button.B)) {
            motorBR.set(1.0);
        } else if(buttonSignal(GamepadKeys.Button.X)) {
            motorFL.set(1.0);
        } else if(buttonSignal(GamepadKeys.Button.Y)) {
            motorFR.set(1.0);
        } else{
            motorFL.set(0);
            motorBL.set(0);
            motorBR.set(0);
            motorFR.set(0);
        }
    }
}
