package org.firstinspires.ftc.teamcode.c_drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.GlobalConfig.SensorFusionValues;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.Encoder;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.Objects;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;

public class SensorFusionLocalizer implements Localizer {
  public Localizer[] localizers = new Localizer[5];
  public double[] positionMultipliers = SensorFusionValues.sensorFusionPositionWeights,
      headingMultipliers = SensorFusionValues.sensorFusionHeadingWeights;
  public double positionSum, headingSum;
  private Encoder rightEncoder, leftEncoder, centerEncoder;
  private Pose2d rightPose, leftPose, centerPose;
  private SensorFusionData data;

  public SensorFusionLocalizer(HardwareMap hardwareMap, BNO055IMU imu1, BNO055IMU imu2){
    rightPose = GlobalConfig.EncoderValues.sideEncoder;
    leftPose = GlobalConfig.EncoderValues.sideEncoder.minus(new Pose2d( 0, 2 * GlobalConfig.EncoderValues.sideEncoder.getY(), 0));
    centerPose = GlobalConfig.EncoderValues.centerEncoder;
    
    data = new SensorFusionData(hardwareMap, imu1, imu2);
    //TODO: switch encoders around if this doesnt work
    localizers[0] = new RROdometryLocalizerIMU(leftPose, centerPose, 2, 1, data);
    localizers[1] = new RROdometryLocalizerIMU(leftPose, centerPose, 2, 2, data);
    localizers[2] = new RROdometryLocalizerIMU(rightPose, centerPose, 0, 1, data);
    localizers[3] = new RROdometryLocalizerIMU(rightPose, centerPose, 0, 2, data);
    localizers[4] = new RROdometryLocalizer(hardwareMap);

    positionSum = DoubleStream.of(positionMultipliers).sum();
    headingSum = DoubleStream.of(headingMultipliers).sum();
  }

  @NotNull
  @Override
  public Pose2d getPoseEstimate() {
    Pose2d sum = IntStream.range(0, localizers.length)
        .mapToObj(index -> {
          Pose2d pose = localizers[index].getPoseEstimate();
          return new Pose2d(pose.vec().times(positionMultipliers[index]),
              pose.getHeading() * headingMultipliers[index]);
        })
        .reduce(Pose2d::plus).orElse(localizers[4].getPoseEstimate());
    Pose2d mean = new Pose2d(
        sum.vec().div(positionSum),
        sum.getHeading() / headingSum);
    //TODO find a way to get this to work
//    for(Localizer l: localizers){
//        l.setPoseEstimate(mean);
//      }
    return mean;
  }

  @Nullable
  @Override
  public Pose2d getPoseVelocity() {
    Pose2d sum = IntStream.range(0, localizers.length)
        .mapToObj(index -> {
          Pose2d pose = localizers[index].getPoseVelocity();
          if (pose == null)
            return null;
          return new Pose2d(pose.vec().times(positionMultipliers[index]),
              pose.getHeading() * headingMultipliers[index]);
        })
        .filter(Objects::nonNull)
        .reduce(Pose2d::plus)
        .orElse(localizers[4].getPoseVelocity());
    Pose2d mean = new Pose2d(
        sum.vec().div(positionSum),
        sum.getHeading() / headingSum);

    return mean;
  }

  @Override
  public void update() {
    data.update();
    for (Localizer l : localizers) {
      l.update();
    }
  }

  @Override
  public void setPoseEstimate(@NotNull Pose2d pose2d) {
    for (Localizer l : localizers) {
      l.setPoseEstimate(pose2d);
    }
  }

  public Pose2d[] getAllPoseEstimates(){
    return Arrays.stream(localizers)
        .map(Localizer::getPoseEstimate)
        .toArray(Pose2d[]::new);
  }
}
