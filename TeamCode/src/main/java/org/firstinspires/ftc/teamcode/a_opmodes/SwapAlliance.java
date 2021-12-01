package org.firstinspires.ftc.teamcode.a_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;

@Autonomous(name = "Swap Alliance", group = "Competition")
public class SwapAlliance extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GlobalConfig.alliance = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? GlobalConfig.Alliance.BLUE : GlobalConfig.Alliance.RED;
        Bot.instance = null;
        telemetry.addLine(GlobalConfig.alliance + " is now sus");
        telemetry.update();
        waitForStart();
    }
}
