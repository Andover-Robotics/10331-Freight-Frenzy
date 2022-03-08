package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {//required vars here
  private double cycle = 0;
  private double prevRead = 0;
  private TimingScheduler timingScheduler;
  private boolean centricity = false;
  private boolean isManual = true;
  private int percent = 1, part = 0;
 // private boolean clawIsOpen = false;




  //config? stuff here =========================================================================

  private double fieldCentricOffset = -90.0;
  public enum TemplateState{
    INTAKE(0.5),
    TRANSPORT(0.5),
    OUTTAKE(0.5);

    public final double progressRate;

    TemplateState(double progressRate){this.progressRate = progressRate;}
  }




  //opmode vars here ==============================================================================================
  //If there is a module-specific var, put it in the module class ie slideStage goes in the slides module


//  private MotorEx carousel;


  void subInit() {
    //TODO: initialize subsystems not initialized in bot constructor
//    timingScheduler = new TimingScheduler(this);
//    carousel = new MotorEx(hardwareMap, "carousel");
//    carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//    carousel.set(0);
   // bot.outtake.clamp();

  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0 / (time - prevRead);
    prevRead = time;
    //timingScheduler.run();
    long profileStart = System.currentTimeMillis();

    //Movement =================================================================================================
    //TODO: change depending on mode :)
    driveSpeed = 1 - 0.1 * (triggerSignal(Trigger.LEFT_TRIGGER) + triggerSignal(Trigger.RIGHT_TRIGGER));

    if (justPressed(Button.BACK)) {
      isManual = !isManual;
    }

    if (isManual) {
      drive();
    } else {
      followPath();
    }


    //TODO: insert actual teleop stuff here
//    if(buttonSignal(Button.DPAD_UP)){
//      bot.carousel.run();
//    }else{
//      bot.carousel.stop();
//    }
//




    if (gamepadEx2.getButton(Button.Y)) {
      bot.intake.run();
    }else if (gamepadEx2.getButton(Button.X)) {
      bot.intake.spit();
    }else{
      bot.intake.stop();
    }

    if (gamepadEx2.getButton(Button.B)) {
      bot.outtake.clamp();
    }else if (gamepadEx2.getButton(Button.A)) {
      bot.outtake.open();
    }

    //carousel
    if(gamepadEx1.getButton(Button.A)){
      bot.carousel.run();
    }else{
      bot.carousel.stop();
    }

    //flip buckets
    if(gamepadEx2.getButton(Button.LEFT_BUMPER)) {
      bot.intake.flipBucket();
      bot.outtake.open();
      bot.intake.spit();
    }else if (gamepadEx2.getButton(Button.RIGHT_BUMPER)){
      bot.intake.unflipBucket();
    }


    //positions for outtake arm
    if (gamepadEx2.getButton(Button.DPAD_LEFT)){
      bot.outtake.runToMid();
    }
    if (gamepadEx2.getButton(Button.DPAD_UP)){
      bot.outtake.runToHigh();
    }
    if (gamepadEx2.getButton(Button.DPAD_RIGHT)) {
      bot.outtake.runToLow();
    }
    if (gamepadEx2.getButton(Button.DPAD_DOWN)){
      bot.outtake.restArm();
    }



  //move outtake arm by itself
    if(gamepadEx2.getTrigger(Trigger.RIGHT_TRIGGER) > 0.01){
      bot.outtake.runArm(gamepadEx2.getTrigger(Trigger.RIGHT_TRIGGER));
    }else if(gamepadEx2.getTrigger(Trigger.LEFT_TRIGGER) > 0.01){
      bot.outtake.runArm(-gamepadEx2.getTrigger(Trigger.LEFT_TRIGGER));
    }else{
      bot.outtake.stopArm();
    }


    if(gamepadEx2.getButton(Button.RIGHT_STICK_BUTTON)) {
      bot.outtake.runArmSlow();
    }


    if(gamepadEx2.getButton(Button.LEFT_STICK_BUTTON)) {
      bot.outtake.downArmSlow();
    }



    /*//TODO: make control scheme
    Controller 1
    A:      B:      X:      Y:
    DPAD
    L:      D:     U:      R:
    Joystick
    L:Field centric movement
    R:Set orientation / Rotation (Determine through practice)
    Trigger L/R: slow driving
    Bumper
    L:none/switch to previous path      R:none/switch to next path
    Other
    Start:  Back:switch between automation and driving

    Controller 2
    A: Carousel    B: Close Claw       X: Intake (take in) & open claw     Y: Intake (spit out)
    DPAD
    L: Arm: Middle Level      D: Arm: rest Position    U: Arm: High Level      R: Arm: Low Level
    Joystick
    L:
    R:
    Trigger
    L: outtake arm thing down (clockwise) R:move outtake arm thing up (Counter clockwise)
    Bumper:
    L: flip bucket                 R: unflip bucket

    Other
    Start:  Back:switch between automation and driving
     */


    /*
    AUTOMATION CONTROL SCHEME

     */



    CommandScheduler.getInstance().run();

    // TODO organize this test code
    updateLocalization();

    telemetry.addData("telemetry things update", System.currentTimeMillis() - profileStart);

    telemetry.addData("percent", percent);
    telemetry.addData("part", part);
    telemetry.addData("cycle", cycle);
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);
   // telemetry.addData("sensor", bot.intake.isPressedBucket());

    telemetry.addData("color sensor value", bot.intake.bucketSensor.alpha());
//    telemetry.addData("is freight in claw: ", bot.intake.isFreightInClaw());
//    telemetry.addData("alpha value: ", bot.intake.alphaValue());
  }


  private void drive(){//Driving ===================================================================================
    updateState();
    double slowModeSpeed = 0.5;

    final double gyroAngle =
        bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).secondAngle//TODO: make sure that the orientation is correct
            - fieldCentricOffset;
    Vector2d driveVector = new Vector2d(gamepadEx1.getLeftX(), gamepadEx1.getLeftY()),
        turnVector = new Vector2d(
            gamepadEx1.getRightX() * Math.abs(gamepadEx1.getRightX()),
            0);
    if (bot.roadRunner.mode == Mode.IDLE) {

      if (centricity) //epic java syntax
        bot.drive.driveFieldCentric(
                driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed,
                gyroAngle);

      else if (gamepadEx1.getTrigger(Trigger.LEFT_TRIGGER) > 0.01 || gamepadEx1.getTrigger(Trigger.RIGHT_TRIGGER) > 0.01)
        bot.drive.driveRobotCentric(
                driveVector.getX() * slowModeSpeed,
                driveVector.getY() * slowModeSpeed,
                turnVector.getX() * slowModeSpeed
        );

      else
        bot.drive.driveRobotCentric(
                driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed
        );

    }

  }

  private void followPath(){//Path following ===================================================================================

    updateState();

  }

  private void updateState(){

  }

  private void updateLocalization() {
    bot.roadRunner.update();
  }
}
