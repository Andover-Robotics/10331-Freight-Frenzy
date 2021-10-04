package org.firstinspires.ftc.teamcode.drive.localizer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Encoder.Direction;

public class SensorFusionData {
  private final Encoder left, center, right;
  private final BNO055IMU imu1, imu2;

  public final int[] positions = new int[3];
  public final double[] velocities = new double[3];
  public double heading1, heading2;

  public SensorFusionData(HardwareMap map, BNO055IMU i1, BNO055IMU i2) {
    left = new Encoder(map.get(DcMotorEx.class, "intake"));
    right = new Encoder(map.get(DcMotorEx.class, "rightEncoder"));
    center = new Encoder(map.get(DcMotorEx.class, "motorFL"));

    left.setDirection(Direction.FORWARD);
    right.setDirection(Direction.FORWARD);
    center.setDirection(Direction.REVERSE);
    imu1 = i1;
    imu2 = i2;
  }

  public void update() {
    positions[0] = left.getCurrentPosition();
    positions[1] = center.getCurrentPosition();
    positions[2] = right.getCurrentPosition();

    velocities[0] = left.getCorrectedVelocity();
    velocities[1] = center.getCorrectedVelocity();
    velocities[2] = right.getCorrectedVelocity();

    heading1 = imu1.getAngularOrientation().firstAngle;
    heading2 = imu2.getAngularOrientation().firstAngle;
  }
}
