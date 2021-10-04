package org.firstinspires.ftc.teamcode.hardware.oldSystemsForReference;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

public class Shooter extends SubsystemBase {
  public static final double onPower = 28.0 * 3700.0 / 60.0, offPower = 28.0 * 0 / 60.0;//changed from 0.5 / 0.25 earlier
  public static final double magazineBackward = 0.74, magazineForward = 0.54;
  public static final double magazineFlipperDown = 0, magazineFlipperUp = 0.25;

  // 5400 rev/min * 1/60 min/second * 28 ticks/rev => ticks/second
  private static int MAX_TICKS_PER_SECOND = 5400 / 60 * 28;

  public static long timerLength = 180;
  private MotorEx motor;
  private Servo magazine, magazineFlipper;
  private Timer feederTimer = null;
  private double targetFlywheelTicksPerSecond = 0.0;

  private static FtcDashboard dash;

  public Shooter(OpMode opMode){
    reset(opMode);
//    motor.setRunMode(RunMode.VelocityControl);
    // using built-in velocity controller for now because FTCLib 1.2.0-beta is broken

    magazine = opMode.hardwareMap.servo.get("magazine");
    magazine.setDirection(Direction.FORWARD);
    magazine.setPosition(magazineBackward);

    magazineFlipper = opMode.hardwareMap.servo.get("magazineFlipper");
    magazineFlipper.setDirection(Direction.FORWARD);
    magazineFlipper.setPosition(magazineFlipperDown);

    dash = FtcDashboard.getInstance();
  }

  public void flipUp(){
    magazineFlipper.setPosition(magazineFlipperUp);
  }

  public void flipDown(){
    magazineFlipper.setPosition(magazineFlipperDown);
  }

  public void feedRing() {
    magazine.setPosition(magazineForward);
    feederTimer = new Timer(timerLength, TimeUnit.MILLISECONDS);
    feederTimer.start();
  }

  @Override
  public void periodic() {
    if (feederTimer != null)
      if (feederTimer.done()) {
        magazine.setPosition(magazineBackward);
        feederTimer = null;
      }
    dash.getTelemetry().addData("shooter velocity", motor.getVelocity());
      dash.getTelemetry().addData("shooter desired power", targetFlywheelTicksPerSecond);
      dash.getTelemetry().addData("shooter power", motor.motorEx.getPower());
      dash.getTelemetry().addData("shooter current", motor.motorEx.getCurrent(CurrentUnit.AMPS));
//    Pair<ExpansionHubEx, ExpansionHubEx> hubs = Bot.getInstance().hubs;
//    if (hubs != null) {
//      if (Math.abs(motor.getVelocity() - targetFlywheelTicksPerSecond) < 80) {
//        hubs.first.setLedColor(5, 232, 225);
//        hubs.second.setLedColor(5, 232, 225);
//      } else {
//        hubs.first.setLedColor(255, 0, 0);
//        hubs.second.setLedColor(255, 0, 0);
//      }
//    }
  }

  public void runShootingSpeed(){
    targetFlywheelTicksPerSecond = onPower;
    motor.motorEx.setVelocity(onPower);
  }

  public void runShootingSpeed(double rpm) {
    targetFlywheelTicksPerSecond = rpm * 28.0 / 60.0;
    motor.motorEx.setVelocity(targetFlywheelTicksPerSecond);
  }

  public void warmUp(double target) {
    motor.motorEx.setVelocity((int) Math.round(MAX_TICKS_PER_SECOND * target));
  }

  public void runIdleSpeed(){
    targetFlywheelTicksPerSecond = offPower;
    motor.motorEx.setVelocity(offPower); // this was the solution --- move setVelocity out of periodic and into the state transition functions
  }

  public void turnOff() {
    motor.motorEx.setVelocity(0);
  }
  // Autonomous functions

  public void shootRings(LinearOpMode opMode, int numRings, double vel) {
    motor.motorEx.setMode(RunMode.RUN_USING_ENCODER);
    motor.motorEx.setVelocity((int) Math.round(MAX_TICKS_PER_SECOND * vel));
    double spinupTime = opMode.getRuntime();
    while (!opMode.isStopRequested() &&
        Math.abs(motor.motorEx.getVelocity() - (int) Math.round(MAX_TICKS_PER_SECOND * vel)) > 400 &&
    opMode.getRuntime() - spinupTime < 0.5) {}

    for (int i = 0; i < numRings; i++) {
      shootOneRing(opMode, vel);
      if (opMode.isStopRequested()) return;
    }
  }

  public void shootOneRing(LinearOpMode opMode, double vel) {
    // intent: pivot the feeder, wait for a moment, wait for the shooter rpm to rebound, pivot the feeder back
    magazine.setPosition(magazineForward);
    opMode.sleep(190);
    magazine.setPosition(magazineBackward);
    opMode.sleep(190);
  }

  public boolean isFlywheelAtTargetVelocity(double target) {
    // tolerance: 5 rev/min * 1/60 min/sec * 50 ticks/rev
    return Math.abs(MAX_TICKS_PER_SECOND * target - motor.getVelocity()) < 4.0 / 60 * 50;
  }

  public void reset(OpMode opMode) {
    motor = new MotorEx(opMode.hardwareMap, "shooter", GoBILDA.RPM_1150);
    motor.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
    motor.motorEx.setMode(RunMode.RUN_USING_ENCODER);
    motor.motorEx.setVelocityPIDFCoefficients(23.5, 2.5, 0.15, 0);
  }

  private int feedGapDuration = 250;
  private int feedDuration = 250;
  private double feederStartTime = 0;

  public final void feederLoop(double runTime) {
    double dt = (runTime - feederStartTime) * 1000;
    double iterDt = dt % (feedGapDuration + feedDuration);
    magazine.setPosition(iterDt >= feedDuration ? magazineBackward : magazineForward);
  }

  public void startFeederLoop(double runTime) {
    feederStartTime = runTime;

  }

  public void bringArmBack(){
    magazine.setPosition(magazineBackward);
  }
}
