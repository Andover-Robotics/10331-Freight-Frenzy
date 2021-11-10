package org.firstinspires.ftc.teamcode.c_drive.localizer;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalConfig.EncoderValues;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.Encoder;

import java.util.Arrays;
import java.util.List;

public class RROdometryLocalizer extends ThreeTrackingWheelLocalizer {

  // This is rev through bore encoder now
  public static double TICKS_PER_REV = 8192;
  public static double WHEEL_RADIUS = 38.75 / 2 / 25.4; // in
  public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
  public static double CENTER_MULT = 1;

  private final Encoder leftEncoder, rightEncoder, centerEncoder;

  public RROdometryLocalizer(HardwareMap hardwareMap) {
    this(hardwareMap, EncoderValues.sideEncoder, EncoderValues.centerEncoder);
  }

  public RROdometryLocalizer(HardwareMap hardwareMap, Pose2d sidePose, Pose2d centerPose) {
    // First calculated in https://docs.google.com/document/d/1s6HzvajxItlIaULulVud0IRhnVrH16yjvsGW4jbwosQ/edit
    super(Arrays.asList(
        new Pose2d(sidePose.getX(), sidePose.getY(), sidePose.getHeading()), // left (relative to front)
        new Pose2d(sidePose.getX(), -sidePose.getY(), sidePose.getHeading()), // right (relative to front)
        centerPose // center
    ));

    leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderValues.leftEncoderPort));
    rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderValues.rightEncoderPort));
    centerEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderValues.centerEncoderPort));

    leftEncoder.setDirection(EncoderValues.leftEncoderDirection);
    rightEncoder.setDirection(EncoderValues.rightEncoderDirection);
    centerEncoder.setDirection(EncoderValues.centerEncoderDirection);
  }

  public static double encoderTicksToInches(double ticks) {
    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
  }

  @NonNull
  @Override
  public List<Double> getWheelPositions() {
    return Arrays.asList(
        encoderTicksToInches(leftEncoder.getCurrentPosition()),
        encoderTicksToInches(rightEncoder.getCurrentPosition()),
        encoderTicksToInches(CENTER_MULT * centerEncoder.getCurrentPosition())
    );
  }

  @NonNull
  @Override
  public List<Double> getWheelVelocities() {
    return Arrays.asList(
        encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
        encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
        encoderTicksToInches(CENTER_MULT * centerEncoder.getCorrectedVelocity())
    );
  }

  @Override
  public void setPoseEstimate(@NonNull Pose2d value) {
    super.setPoseEstimate(value);
  }
}
