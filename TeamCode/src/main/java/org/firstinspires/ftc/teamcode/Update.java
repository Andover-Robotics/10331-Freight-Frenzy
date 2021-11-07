package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Update extends LinearOpMode {
  //TODO: look into openftc/opmode configurer
  @Override
  public void runOpMode() throws InterruptedException {
    telemetry.addData("update",
        432123
    );//change value here and see if it changes on telemetry
    telemetry.update();
    waitForStart();
  }
}
