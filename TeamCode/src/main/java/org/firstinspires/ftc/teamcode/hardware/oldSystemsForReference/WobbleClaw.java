package org.firstinspires.ftc.teamcode.hardware.oldSystemsForReference;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Differentiator;

public class WobbleClaw extends SubsystemBase {
  public static double
      clawClosedPos = 0.28, clawOpenPos = 0.55, armSpeed = 0.35, ffMaxPower = 0.27, ffZeroAngle = Math.toRadians(4);
  public static int
      armUpPos = 0, armDownPos = 370, armDropPos = 300, armDropEndgamePos = 220;
  private int armPos = 0;//-156 is position @ "top"

  public DcMotorEx armRotator;
  private PIDFController armController;
  private ArmFeedforward armFF;
  private Differentiator armSpeedDiff;
  private Servo claw;
  private boolean powerArm = false;

  public WobbleClaw(OpMode opMode) {
    armRotator = opMode.hardwareMap.get(DcMotorEx.class, "wobbleArm");
    armRotator.setTargetPosition(0);
    armRotator.setMode(RunMode.RUN_TO_POSITION);
    // last: 32
    armRotator.setVelocityPIDFCoefficients(28, 1.5, 1.5, 0.2);
    armRotator.setTargetPositionTolerance(80);
    armRotator.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    armRotator.setDirection(Direction.REVERSE);
    armPos = armRotator.getCurrentPosition();
//    armFF = new ArmFeedforward(0.05, ffMaxPower - 0.05, 0.05);
//    armSpeedDiff = new Differentiator(getArmPosRadians());
//    armController = new PIDFController(0.03, 0.008, 0.02, 1);
//    armController.setTolerance(30, 20);
//    armController.setIntegrationBounds(0, 100);

    claw = opMode.hardwareMap.servo.get("claw");
    dash = FtcDashboard.getInstance();
  }

  public void stopArm() {
    powerArm = false;
    armRotator.setPower(0);
  }

  public void open() {
    claw.setPosition(clawOpenPos);
  }

  public void close() {
    claw.setPosition(clawClosedPos);
  }


  private FtcDashboard dash;

  public void periodic() {
//    if (powerArm) {
//      double armAngle = getArmPosRadians();
//      double armVel = armSpeedDiff.update(armAngle);
//      double feedforward = armFF.calculate(armAngle, armVel);
//
//      armController.setF(feedforward);
//      double output = armController.calculate(armRotator.getCurrentPosition());
//      armRotator.setPower(output);
//
//      TelemetryPacket packet = new TelemetryPacket();
//      packet.put("armAngle", armAngle);
//      packet.put("armFF", feedforward);
//      packet.put("armOutput", output);
//      dash.sendTelemetryPacket(packet);
//    }
    dash.getTelemetry().addData("arm target", armRotator.getTargetPosition());
    dash.getTelemetry().addData("arm position", armRotator.getCurrentPosition());
    dash.getTelemetry().addData("arm power", armRotator.getPower());
    if (armRotator.getMode() != RunMode.RUN_WITHOUT_ENCODER &&
        armRotator.getTargetPosition() == 0 && armRotator.getCurrentPosition() < 50){
      armRotator.setMode(RunMode.RUN_WITHOUT_ENCODER);
      stopArm();
    }
  }

  private double getArmPosRadians() {
    return Math.toRadians(armRotator.getCurrentPosition()) + ffZeroAngle;
  }

  public void raiseArm() {
    runArmTo(armUpPos);
  }

  public void lowerArmToDrop() {
    runArmTo(armDropPos);
  }

  public void lowerArmToDropInEndgame() {
    runArmTo(armDropEndgamePos);
  }

  public void lowerArm() {
    runArmTo(armDownPos);
  }

  public void stowArm() {
    runArmTo(0);
  }

  private void runArmTo(int pos) {
//    armController.reset();
//    armController.setSetPoint(pos);
//    powerArm = true;
    armRotator.setTargetPosition(pos);
    armRotator.setMode(RunMode.RUN_TO_POSITION);
    armRotator.setPower(0.28);
  }


  // Autonomous specific
  public void waitUntilTargetReached(LinearOpMode opMode) {
    double startTime = opMode.getRuntime();
    while (Math.abs(armRotator.getCurrentPosition() - armRotator.getTargetPosition()) > 20 ||
        armRotator.isBusy() && !opMode.isStopRequested()) {
      if (opMode.getRuntime() - startTime > 3) break;
    }
  }
}
