package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.b_hardware.Bot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

/**
 * Make a TeleOp OpMode for a robot with the following hardware devices:
 *  - Mecanum drive base (motorFL, motorFR, motorBL, motorBR)
 *  - Intake roller (intake)
 *  - Shooter flywheel (shooter, full speed launches rings)
 *
 *  Your OpMode should enable the driver to drive the robot holonomically (i.e. moving in any direction),
 *  operate the intake, and operate the shooter. Bonus if you can automate some of this.
 *
 *  You design the control scheme.
 */                                           //    -Michael, Fall 2020
public abstract class BaseOpMode extends OpMode {

  Bot bot;
  double driveSpeed;
  GamepadEx gamepadEx1, gamepadEx2;

  //button reader syntax
  // (g1 or g2)  (a, b, lt, lb, etc)

  @Override
  public void init() {
    bot = Bot.getInstance(this);
    gamepadEx2 = new GamepadEx(gamepad2);
    gamepadEx1 = new GamepadEx(gamepad1);
    subInit();
    telemetry.addLine("Init done");
    telemetry.update();
  }

  @Override
  public void loop() {
    updateButtons();
    subLoop();
  }

  abstract void subInit();

  abstract void subLoop();

  void updateButtons(){
    gamepadEx1.readButtons();
    gamepadEx2.readButtons();
  }



  boolean buttonSignal(Button button) {
    return gamepadEx1.isDown(button) || gamepadEx2.isDown(button);
  }

  double triggerSignal(Trigger trigger) {
    double in1 = gamepadEx1.getTrigger(trigger),
        in2 = gamepadEx2.getTrigger(trigger);
    return Math.max(in1, in2);
  }

  Vector2d stickSignal(Direction side) {
    Function<GamepadEx, Vector2d> toCoords = pad ->
        side == Direction.LEFT ? new Vector2d(pad.getLeftX(), pad.getLeftY()) :
            new Vector2d(pad.getRightX(), pad.getRightY());

    Vector2d v1 = toCoords.apply(gamepadEx1),
        v2 = toCoords.apply(gamepadEx2);

    return v1.magnitude() > 0.02 ? v1 : v2;
  }

  boolean justPressed(Button button) {
    return gamepadEx1.wasJustPressed(button) || gamepadEx2.wasJustPressed(button);
  }

  boolean justReleased(Button button){
    return !(gamepadEx1.isDown(button) || gamepadEx2.isDown(button)) && (gamepadEx1.wasJustReleased(button) || gamepadEx2.wasJustReleased(button));
  }
}
