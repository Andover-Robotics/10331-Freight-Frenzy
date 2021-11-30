package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Intake;
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
  private boolean isLeftIntakeRunning = false;




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





  void subInit() {
    //TODO: initialize subsystems not initialized in bot constructor
    timingScheduler = new TimingScheduler(this);
  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();
//    long profileStart = System.currentTimeMillis();

    //Movement =================================================================================================
    //TODO: change depending on mode :)
    driveSpeed = 1 - 0.35 * (triggerSignal(Trigger.LEFT_TRIGGER) + triggerSignal(Trigger.RIGHT_TRIGGER));

    if(justPressed(Button.BACK)){
      isManual = !isManual;
    }

    if(isManual) {
      drive();
    }else{
      followPath();
    }


    //TODO: insert actual teleop stuff here
//    if(buttonSignal(Button.DPAD_UP)){
//      bot.carousel.run();
//    }else{
//      bot.carousel.stop();
//    }
//
    //clicking left intake
    if(gamepadEx2.stateJustChanged(Button.B) && bot.intake.runState == Intake.STATE.OFF){
      bot.intake.runLeft();
      bot.gate.openLeft();
      bot.gate.takeInRight();
    }
    else if(gamepadEx2.stateJustChanged(Button.B) && bot.intake.runState == Intake.STATE.LEFT){
      bot.intake.stop();
//      bot.gate.closeLeft();
//      bot.gate.closeRight();
    }


    //hold down button for intake
    if(gamepadEx2.isDown(Button.B)){
      bot.intake.runLeft();
    }
    else{
      bot.intake.stop();
      bot.gate.closeLeft();
      bot.gate.closeRight();
    }

//    telemetry.addData("left intake update", System.currentTimeMillis() - profileStart);



    //clicking right intake
//    if(gamepadEx2.stateJustChanged(Button.X) && bot.intake.runState == Intake.STATE.OFF){
//      bot.intake.runRight();
//      bot.gate.openRight();
//      bot.gate.takeInLeft();
//    }
//    else if(gamepadEx2.stateJustChanged(Button.X) && bot.intake.runState == Intake.STATE.RIGHT){
//      bot.intake.stop();
////      bot.gate.closeRight();
////      bot.gate.closeLeft();
//    }

//    hold down button for intake
    if(gamepadEx2.isDown(Button.X)){
      bot.gate.openRight();
      bot.gate.takeInLeft();
      bot.intake.runRight();
    }
    else{
      bot.intake.stop();
      bot.gate.closeRight();
      bot.gate.closeLeft();
    }

//    telemetry.addData("right intake update", System.currentTimeMillis() - profileStart);
    //hold down button for carousel
    if(gamepadEx2.isDown(Button.Y)){
      bot.carousel.run();
    }
    else{
      bot.carousel.stop();
    }

//    telemetry.addData("carousel update", System.currentTimeMillis() - profileStart);

//
    if(gamepadEx2.isDown(Button.A)){
      bot.carousel.reverse();
    }
    else{
      bot.carousel.stop();
    }



//    //clicking gate
//    if(gamepadEx1.stateJustChanged(Button.A) && bot.gate.runState == Gate.State.OFF){
//      bot.gate.closeGateFlap();
//    }
//    else if(gamepadEx1.stateJustChanged(Button.A) && bot.gate.runState == Gate.State.ON){
//      bot.gate.openGateFlap();
//    }
//
//    //hold down button
//    if(gamepadEx1.isDown(Button.A)){
//      bot.gate.closeGateFlap();
//    }
//    else{
//      bot.gate.openGateFlap();
//    }

    //hold down triggers for outtake
    if(gamepadEx2.getTrigger(Trigger.LEFT_TRIGGER)>0.01){
      bot.outtake.up();
    }
    else {
      bot.outtake.stop();
    }

//    telemetry.addData("outtake up update", System.currentTimeMillis() - profileStart);


    if (gamepadEx2.getTrigger(Trigger.RIGHT_TRIGGER)>0.01){
      bot.outtake.down();
    }
    else{
      bot.outtake.stop();
    }

//    telemetry.addData("outtake down update", System.currentTimeMillis() - profileStart);






    /*//TODO: make control scheme
    Controller 1
    A:      B:      X:      Y:
    DPAD
    L:      D:     U:      R:
    Joystick
    L:Field centric movement
    R:Set orientation / Rotation (Determine through practice)
    Trigger L/R: slow driving (maybe)
    Bumper
    L:none/switch to previous path      R:none/switch to next path
    Other
    Start:  Back:switch between automation and driving

    Controller 2
    A:      B:      X:      Y:
    DPAD
    L:      D:     U:      R:
    Joystick
    L:movement/reset field centric or progress automation
    R:movement/switch robotfield centric or none
    Trigger L/R: slow driving
    Bumper
    L:none/switch to previous path      R:none/switch to next path
    Other
    Start:  Back:switch between automation and driving
     */


    /*
    AUTOMATION CONTROL SCHEME

     */



    CommandScheduler.getInstance().run();

    // TODO organize this test code
    updateLocalization();

//    telemetry.addData("telemetry things update", System.currentTimeMillis() - profileStart);

    telemetry.addData("percent", percent);
    telemetry.addData("part", part);
    telemetry.addData("cycle", cycle);
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);
  }


  private void drive(){//Driving ===================================================================================
    updateState();

    final double gyroAngle =
        bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).secondAngle//TODO: make sure that the orientation is correct
            - fieldCentricOffset;
    Vector2d driveVector = stickSignal(Direction.LEFT),
        turnVector = new Vector2d(
            stickSignal(Direction.RIGHT).getX() * Math.abs(stickSignal(Direction.RIGHT).getX()),
            0);
    if (bot.roadRunner.mode == Mode.IDLE) {
      if (centricity)//epic java syntax
        bot.drive.driveFieldCentric(
            driveVector.getX() * driveSpeed,
            driveVector.getY() * driveSpeed,
            turnVector.getX() * driveSpeed,
            gyroAngle);
      else
        bot.drive.driveRobotCentric(
            driveVector.getX() * driveSpeed,
            driveVector.getY() * driveSpeed,
            turnVector.getX() * driveSpeed
        );
    }
    if (justPressed(Button.LEFT_STICK_BUTTON)) {
      fieldCentricOffset = bot.imu.getAngularOrientation()
          .toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }
    if(justPressed(Button.RIGHT_STICK_BUTTON)){
      centricity = !centricity;
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
