package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Update extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    telemetry.addData("update",
        4321900
    );//change value here and see if it changes on telemetry
    telemetry.update();
    waitForStart();
  }
}
