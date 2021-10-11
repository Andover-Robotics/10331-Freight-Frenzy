package org.firstinspires.ftc.teamcode.drive.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import java.util.Arrays;
import java.util.List;

public class RROdometryLocalizerIMU extends TwoTrackingWheelLocalizer {

  // This is rev through bore encoder now
  public static double TICKS_PER_REV = 8192;
  public static double WHEEL_RADIUS = 38.75 / 2 / 25.4; // in
  public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
  public static double CENTER_MULT = 24 / 23.55;

  private int encoder1, encoder2 = 1, imu;
  private final SensorFusionData dataSource;

  public RROdometryLocalizerIMU(Pose2d pose1, Pose2d centerPose,
      int encoder1, int imuNum, SensorFusionData data) {
    super(Arrays.asList(pose1, centerPose));
    this.encoder1 = encoder1;
    this.imu = imuNum;
    dataSource = data;
  }

  public static double encoderTicksToInches(double ticks) {
    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
  }

  @NonNull
  @Override
  public List<Double> getWheelPositions() {
    return Arrays.asList(
        encoderTicksToInches(dataSource.positions[encoder1]),
        encoderTicksToInches(CENTER_MULT * dataSource.positions[encoder2])
    );
  }

  @NonNull
  @Override
  public List<Double> getWheelVelocities() {
    return Arrays.asList(
        encoderTicksToInches(dataSource.velocities[encoder1]),
        encoderTicksToInches(dataSource.velocities[encoder2])
    );
  }

  @Override
  public double getHeading() {
    return imu == 1 ? dataSource.heading1 : dataSource.heading2;
  }

  @Override
  public void setPoseEstimate(@NonNull Pose2d value) {
    super.setPoseEstimate(value);
  }
}
