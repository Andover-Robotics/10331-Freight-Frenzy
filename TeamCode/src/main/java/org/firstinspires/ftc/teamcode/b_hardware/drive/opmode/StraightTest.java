package org.firstinspires.ftc.teamcode.b_hardware.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.b_hardware.drive.RRMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {

  public static double DISTANCE = 24; // in

  @Override
  public void runOpMode() throws InterruptedException {
    RRMecanumDrive drive = new RRMecanumDrive(hardwareMap);
    Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
        .forward(DISTANCE)
        .build();

    waitForStart();

    if (isStopRequested()) {
      return;
    }

    drive.followTrajectory(trajectory);

    Pose2d poseEstimate = drive.getPoseEstimate();
    telemetry.addData("finalX", poseEstimate.getX());
    telemetry.addData("finalY", poseEstimate.getY());
    telemetry.addData("finalHeading", poseEstimate.getHeading());
    telemetry.update();

    while (!isStopRequested() && opModeIsActive()) {
      ;
    }
  }
}
