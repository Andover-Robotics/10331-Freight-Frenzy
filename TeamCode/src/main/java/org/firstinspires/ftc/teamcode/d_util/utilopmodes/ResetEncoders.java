package org.firstinspires.ftc.teamcode.d_util.utilopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import java.util.Map.Entry;

@Autonomous(name = "Reset Encoders", group = "Utility")
public class ResetEncoders extends LinearOpMode {
  public void runOpMode() {
    telemetry.setAutoClear(false);
    waitForStart();

    for (Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
      entry.getValue().setMode(RunMode.STOP_AND_RESET_ENCODER);
      while (!isStopRequested() && Math.abs(entry.getValue().getCurrentPosition()) > 1) {
        idle();
      }
      telemetry.addData(entry.getKey(), "is reset");
      telemetry.update();
    }

    while (!isStopRequested()) {}
  }
}
