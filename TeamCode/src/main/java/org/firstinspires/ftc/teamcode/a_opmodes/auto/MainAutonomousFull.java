package org.firstinspires.ftc.teamcode.a_opmodes.auto;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.WarehouseSide.AutoPathElement;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.WarehouseSide.AutoPathElement.Action;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.WarehouseSide.AutoPathElement.Path;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.BarcodeDetector;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector.PipelineResult;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

@Autonomous(name = "Main Autonomous Full Side", group = "Competition")
public class MainAutonomousFull extends LinearOpMode {//TODO: add reversing for competition

    private Bot bot;

    OpenCvCamera phoneCam;

    TemplateDetector.PipelineResult detected;
    double confidence;
    TemplateDetector pipeline;
    boolean performActions = true;
    GamepadEx gamepad;



    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);
        gamepad = new GamepadEx(gamepad1);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier ("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());

        phoneCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        BarcodeDetector detector = new BarcodeDetector(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                                       {
                                           @Override
                                           public void onOpened()
                                           {
                                               phoneCam.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_LEFT);
                                           }

                                           @Override
                                           public void onError(int errorCode) {

                                           }

                                       }
                // () -> phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT)
        );



        FullSide paths = new FullSide(this);
//    pipeline = new TemplateDetector(this);

        //TODO: add initialization here

        //  ie set servo position                             ========================================================================


        //Pipeline stuff

//    while (!isStarted()) {
//      if (isStopRequested())
//        return;
//      // keep getting results from the pipeline
//      pipeline.currentlyDetected()
//          .ifPresent((pair) -> {
//            telemetry.addData("detected", pair.first);
//            telemetry.addData("confidence", pair.second);
//            telemetry.update();
//            detected = pair.first;
//            confidence = pair.second;
//          });
//      if (gamepad1.x) {
//        performActions = false;
//      }
//      if (gamepad.wasJustPressed(Button.Y)) {
//        pipeline.saveImage();
//      }
//    }
//
//    pipeline.currentlyDetected().ifPresent(pair -> {
//      detected = pair.first;
//      confidence = pair.second;
//    });
//
//    if (detected == null)
//      detected = PipelineResult.LEFT;

//        detected = TemplateDetector.PipelineResult.RIGHT;

        telemetry.addLine(GlobalConfig.alliance + " is selected alliance");

        telemetry.update();


        waitForStart();

        switch (detector.getPosition()){
            case LEFT:
                detected = PipelineResult.LEFT;
                break;
            case MIDDLE:
                detected = PipelineResult.MIDDLE;
                break;
            case RIGHT:
                detected = PipelineResult.RIGHT;
                break;
            case NOT_FOUND:
                detected = PipelineResult.RIGHT;
                break;
        }

        phoneCam.stopStreaming();

        // List<AutoPathElement> trajectories = paths.getTrajectories (detected);
        List<FullSide.AutoPathElement> trajectories = paths.getTrajectories(detected);
//    pipeline.close();


        //Roadrunner stuff

        bot.roadRunner.setPoseEstimate(paths.getStartPose());

        if (isStopRequested())
            return;

        for (FullSide.AutoPathElement item : trajectories) {

            telemetry.addData("executing path element", item.getName());
            telemetry.update();

            if (item instanceof FullSide.AutoPathElement.Path) {
                bot.roadRunner.followTrajectory(((FullSide.AutoPathElement.Path) item).getTrajectory());
            } else if (item instanceof FullSide.AutoPathElement.Action && performActions) {
                ((FullSide.AutoPathElement.Action) item).getRunner().invoke();
            }

            if (isStopRequested())
                return;
        }
    }


}
