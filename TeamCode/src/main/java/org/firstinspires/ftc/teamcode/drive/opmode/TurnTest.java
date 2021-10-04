package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.localizer.RROdometryLocalizer;

import java.io.FileNotFoundException;
import java.io.PrintWriter;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {

  public static double centerEncoderX = 5.65, centerEncoderY = 0.5;
  public static double sideEncoderX = 0.55, sideEncoderY = 7.3;
  public static double ANGLE = 90; // deg

  private RRMecanumDrive drive;
  private PrintWriter fos;

  @Override
  public void runOpMode() throws InterruptedException {
    drive = new RRMecanumDrive(hardwareMap);
    try {
      fos = new PrintWriter("/sdcard/FIRST/odo-diag.tsv");
      fos.println("Cx\tCy\tLx\tLy\tLh");
    } catch (FileNotFoundException e) {
      e.printStackTrace();
      stop();
    }
    updateLocalizer();

    waitForStart();

    if (isStopRequested()) {
      return;
    }

//    drive.turn(Math.toRadians(ANGLE));
    gridSearchTurn();

while (!isStopRequested()) {}
fos.close();
  }

  public void gridSearchTurn() {
    double previousHeading = 0;
    telemetry.setAutoClear(false);
//    for (double x = 0.53; x <= 0.57; x += 0.02) {
//      for (double y = 5.7; y <= 6.2; y += 0.1) {
//        for(int i = 0; i < 3; i++) {
//          sideEncoderX = x;
//          sideEncoderY = y;
//          updateLocalizer();
//          drive.setPoseEstimate(new Pose2d());
//          drive.turn(Math.PI / 2);
//          telemetry.addData("trial", "Sx=%.3f Sy=%.3f Lx=%.3f, Ly=%.3f Lh=%.3f", x, y,
//              drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),
//              Math.toDegrees(drive.getRawExternalHeading() - previousHeading));
//          fos.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", x, y,
//              drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),
//              Math.toDegrees(drive.getRawExternalHeading() - previousHeading));
//          telemetry.update();
//          previousHeading = drive.getRawExternalHeading();
//        }
//      }
//    }

    for (double x = 5.2; x <= 6.0; x += 0.2) {
      for (double y = 0.5; y <= 0.5; y += 0.1) {
        for(int i = 0; i < 3; i++) {
          centerEncoderX = x;
          centerEncoderY = y;
          updateLocalizer();
          drive.setPoseEstimate(new Pose2d());
          drive.turn(Math.PI / 2);
          telemetry.addData("trial", "Cx=%.3f Cy=%.3f Lx=%.3f, Ly=%.3f Lh=%.3f", x, y,
              drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),
              Math.toDegrees(drive.getRawExternalHeading() - previousHeading));
          fos.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", x, y,
              drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),
              Math.toDegrees(drive.getRawExternalHeading() - previousHeading));
          telemetry.update();
          previousHeading = drive.getRawExternalHeading();
        }
      }
    }
  }

  private void updateLocalizer() {
    drive.setLocalizer(new RROdometryLocalizer(hardwareMap,
        new Pose2d(sideEncoderX, sideEncoderY, 0),
        new Pose2d(centerEncoderX, centerEncoderY, Math.PI/2)));
  }
}
