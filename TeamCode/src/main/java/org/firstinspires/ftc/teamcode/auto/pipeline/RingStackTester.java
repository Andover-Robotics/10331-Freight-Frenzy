package org.firstinspires.ftc.teamcode.auto.pipeline;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class RingStackTester extends OpMode {

  private OpenCvCamera camera;

  @Override
  public void init() {
    Bot bot = Bot.getInstance();
    int cameraMonitorViewId = this.hardwareMap.appContext.getResources()
        .getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
    camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,
        cameraMonitorViewId);
    camera.startStreaming(320 * 3, 240 * 3, OpenCvCameraRotation.SIDEWAYS_LEFT);
  }

  @Override
  public void loop() {
//    telemetry.addData("Size: ", );
//    telemetry.update();
  }
}
