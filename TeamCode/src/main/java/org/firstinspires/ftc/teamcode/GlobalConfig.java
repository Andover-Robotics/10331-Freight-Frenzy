package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Encoder.Direction;

public class GlobalConfig {//make all fields final
  //control + click class to go to it
  DriveConstants controlClickForDriveConstants;

  //TODO add more config stuff
  public static final String motorFL = "motorFL", motorFR = "motorFR", motorBL = "motorBL", motorBR = "motorBR";




  public static class SensorFusionValues{
    public static final double[] sensorFusionHeadingWeights = {0.225, 0.225, 0.225, 0.225, 0.1},
        sensorFusionPositionWeights = {0.15, 0.15, 0.15, 0.15, 0.4};
  }

  public static class EncoderValues{
    public static final Pose2d sideEncoder = new Pose2d(-0.55, 7.3, 0), centerEncoder = new Pose2d(5.6, 0.512, Math.toRadians(90));
    public static final String leftEncoderPort = "intake", rightEncoderPort = "motorFL", centerEncoderPort = "fdas";
    public static final Direction leftEncoderDirection = Direction.FORWARD, rightEncoderDirection = Direction.FORWARD, centerEncoderDirection = Direction.REVERSE;
  }
}
