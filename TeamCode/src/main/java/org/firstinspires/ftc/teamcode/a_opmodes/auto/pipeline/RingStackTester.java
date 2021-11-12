package org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.b_hardware.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2Impl;

public class RingStackTester extends OpMode {

  private OpenCvCamera camera;

  @Override
  public void init() {
    Bot bot = Bot.getInstance();
    int cameraMonitorViewId = this.hardwareMap.appContext.getResources()
        .getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
    camera = new OpenCvInternalCamera2Impl(OpenCvInternalCamera2Impl.CameraDirection.BACK,
        cameraMonitorViewId);
    camera.startStreaming(320 * 3, 240 * 3, OpenCvCameraRotation.SIDEWAYS_LEFT);
  }

  @Override
  public void loop() {
//    telemetry.addData("Size: ", );
//    telemetry.update();
  }
}
