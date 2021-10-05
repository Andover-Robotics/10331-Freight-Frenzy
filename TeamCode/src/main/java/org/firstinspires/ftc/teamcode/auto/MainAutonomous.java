package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutoPaths.AutoPathElement;
import org.firstinspires.ftc.teamcode.auto.AutoPaths.AutoPathElement.Action;
import org.firstinspires.ftc.teamcode.auto.AutoPaths.AutoPathElement.Path;
import org.firstinspires.ftc.teamcode.auto.pipeline.TemplateDetector;
import org.firstinspires.ftc.teamcode.auto.pipeline.TemplateDetector.PipelineResult;
import org.firstinspires.ftc.teamcode.hardware.Bot;

import java.util.List;

@Autonomous(name = "Main Autonomous", group = "Competition")
public class MainAutonomous extends LinearOpMode {

  private Bot bot;

  PipelineResult detected;
  double confidence;
  TemplateDetector pipeline;
  boolean performActions = true;
  GamepadEx gamepad;


  @Override
  public void runOpMode() throws InterruptedException {
    Bot.instance = null;
    bot = Bot.getInstance(this);
    gamepad = new GamepadEx(gamepad1);

    AutoPaths paths = new AutoPaths(this);
    pipeline = new TemplateDetector(this);

    //TODO: add initialization here

    //  ie set servo position                             ========================================================================


    //Pipeline stuff

    while (!isStarted()) {
      if (isStopRequested())
        return;
      // keep getting results from the pipeline
      pipeline.currentlyDetected()
          .ifPresent((pair) -> {
            telemetry.addData("detected", pair.first);
            telemetry.addData("confidence", pair.second);
            telemetry.update();
            detected = pair.first;
            confidence = pair.second;
          });
      if (gamepad1.x) {
        performActions = false;
      }
      if (gamepad.wasJustPressed(Button.Y)) {
        pipeline.saveImage();
      }
    }

    pipeline.currentlyDetected().ifPresent(pair -> {
      detected = pair.first;
      confidence = pair.second;
    });

    if (detected == null)
      detected = PipelineResult.ZERO;
    List<AutoPathElement> trajectories = paths.getTrajectories(detected);
    pipeline.close();


    //Roadrunner stuff

    bot.roadRunner.setPoseEstimate(paths.getStartPose());

    if (isStopRequested())
      return;

    for (AutoPathElement item : trajectories) {

      telemetry.addData("executing path element", item.getName());
      telemetry.update();

      if (item instanceof AutoPathElement.Path) {
        bot.roadRunner.followTrajectory(((Path) item).getTrajectory());
      } else if (item instanceof AutoPathElement.Action && performActions) {
        ((Action) item).getRunner().invoke();
      }

      if (isStopRequested())
        return;
    }
  }
}