package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.c_drive.DriveConstants;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.Encoder.Direction;
@Config
public class GlobalConfig {//make all fields final
  //control + click class to go to it
  DriveConstants controlClickForDriveConstants;
  public enum Alliance{
    RED,
    BLUE;

    @NonNull
    @Override
    public String toString() {
      return super.toString();
    }
  }

  //TODO add more config stuff
  public static final String motorFL = "motorFL", motorFR = "motorFR", motorBL = "motorBL", motorBR = "motorBR";
  public static Alliance alliance = Alliance.BLUE;



  public static class SensorFusionValues{
    public static final double[] sensorFusionHeadingWeights = {0.225, 0.225, 0.225, 0.225, 0},
        sensorFusionPositionWeights = {0.15, 0.15, 0.15, 0.15, 0.4};
  }

  public static class EncoderValues{
    public static final Pose2d sideEncoder = new Pose2d(0.1965, 4.5, 0), centerEncoder = new Pose2d(-5.2, 0.75, Math.toRadians(90));
    public static final String leftEncoderPort = "motorFL", rightEncoderPort = "motorBL", centerEncoderPort = "motorFR";
    public static final Direction leftEncoderDirection = Direction.REVERSE, rightEncoderDirection = Direction.FORWARD, centerEncoderDirection = Direction.REVERSE;
  }
}
