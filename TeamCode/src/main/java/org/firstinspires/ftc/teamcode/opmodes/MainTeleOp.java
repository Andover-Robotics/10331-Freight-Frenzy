package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.util.TimingScheduler;

import java.util.Map;
import java.util.Map.Entry;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {//required vars here
  private double cycle = 0;
  private double prevRead = 0;
  private TimingScheduler timingScheduler;
  private boolean centricity = false;
  private PathFollower follower;
  private boolean isManual = true;
  private int percent = 0, part = 0;




  //config? stuff here =========================================================================

  private double fieldCentricOffset = -90.0;
  public enum TemplateState{
    INTAKE(0.5),
    TRANSPORT(0.5),
    OUTTAKE(0.5);

    public final double progressRate;

    TemplateState(double progressRate){this.progressRate = progressRate;}
  }

  Map<TemplateState, Map<Button, TemplateState>> stateMap = new StateMap().getStateMap();

  public TemplateState state = TemplateState.INTAKE;


  //opmode vars here ==============================================================================================
  //If there is a module-specific var, put it in the module class ie slideStage goes in the slides module




  void subInit() {
    //TODO: initialize subsystems not initialized in bot constructor
    timingScheduler = new TimingScheduler(this);
    follower = new PathFollower(this);
  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();

    //Movement =================================================================================================
    //TODO: change depending on mode
    driveSpeed = 1 - 0.35 * (triggerSignal(Trigger.LEFT_TRIGGER) + triggerSignal(Trigger.RIGHT_TRIGGER));

    if(justPressed(Button.START)){
      isManual = !isManual;
    }

    if(isManual) {
      drive();
    }else{
      followPath();
    }




    //TODO: insert actual teleop stuff here
    if(justPressed(Button.A) || gamepadEx1.wasJustReleased(Button.A)){
      bot.templateSubsystem.operateSlides(1);
    }


    /*//TODO: make control scheme
    Controller 1
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
    Start:switch between automation and driving  Back:

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
    Start:switch between automation and driving  Back:
     */


    /*
    AUTOMATION CONTROL SCHEME

     */



    CommandScheduler.getInstance().run();

    // TODO organize this test code
    updateLocalization();
    telemetry.addData("cycle", cycle);
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);
  }


  private void drive(){//Driving ===================================================================================
    updateState();

    final double gyroAngle =
        bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
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
            -driveVector.getX() * driveSpeed,
            -driveVector.getY() * driveSpeed,
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

    if(!follower.isTrajectory(state, part)){
      drive();
    } else {
      percent += stickSignal(Direction.LEFT).getY() * state.progressRate;
    }
    percent = Math.max(0, Math.min(100, percent));


    if(justPressed(Button.RIGHT_BUMPER) || percent >= 100){
      percent = 0;
      part += 1;
      if(part > follower.getPathsInfo().get(state) - 1){
        part = 0;
        //TODO: add automatic state changer?
      }
    } else if(justPressed(Button.LEFT_BUMPER) || percent <= 0){
      percent = 100;
      part -= 1;
      if(part < 0){
        part = follower.getPathsInfo().get(state) - 1;
      }
    }

    follower.followPath(state, percent, part);

  }

  private void updateState(){
    for(Entry<Button, TemplateState> pair : stateMap.get(state).entrySet()){
      if(justPressed(pair.getKey())){
        state = pair.getValue();
        percent = 0;
      }
    }
  }

  private void updateLocalization() {
    bot.roadRunner.update();
  }
}
