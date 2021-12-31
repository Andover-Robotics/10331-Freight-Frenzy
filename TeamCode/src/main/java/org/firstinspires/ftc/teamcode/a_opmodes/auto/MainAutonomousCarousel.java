package org.firstinspires.ftc.teamcode.a_opmodes.auto;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.CarouselSide.AutoPathElement;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.CarouselSide.AutoPathElement.Action;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.CarouselSide.AutoPathElement.Path;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector.PipelineResult;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;

import java.util.List;

@Autonomous(name = "Main Autonomous Carousel Side", group = "Competition")
public class MainAutonomousCarousel extends LinearOpMode {//TODO: add reversing for competition

    private Bot bot;

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

        CarouselSide paths = new CarouselSide(this);
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

        detected = TemplateDetector.PipelineResult.RIGHT;

        telemetry.addLine(GlobalConfig.alliance + " is selected alliance");

        telemetry.update();


        // List<AutoPathElement> trajectories = paths.getTrajectories (detected);
        List<CarouselSide.AutoPathElement> trajectories = paths.getTrajectories(detected);
//    pipeline.close();


        //Roadrunner stuff

        bot.roadRunner.setPoseEstimate(paths.getStartPose());

        if (isStopRequested())
            return;

        for (CarouselSide.AutoPathElement item : trajectories) {

            telemetry.addData("executing path element", item.getName());
            telemetry.update();

            if (item instanceof CarouselSide.AutoPathElement.Path) {
                bot.roadRunner.followTrajectory(((CarouselSide.AutoPathElement.Path) item).getTrajectory());
            } else if (item instanceof CarouselSide.AutoPathElement.Action && performActions) {
                ((CarouselSide.AutoPathElement.Action) item).getRunner().invoke();
            }

            if (isStopRequested())
                return;
        }
    }


}


