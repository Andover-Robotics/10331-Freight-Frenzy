package org.firstinspires.ftc.teamcode.d_util.toolbox;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

@TeleOp(name = "TBX: Servo Diagnostic", group = "ARC Toolbox")
public class ServoDiagnostic extends OpMode {
  private static double POS_DELTA = 0.05;

  private Servo servo;
  private InputColumnResponder input = new InputColumnResponderImpl();
  private Selector servoSelector;

  @Override
  public void init() {
    servoSelector = new Selector(hardwareMap.servo.entrySet().stream().map(Map.Entry::getKey));
    input.register(() -> gamepad1.x, servoSelector::selectNext);
  }

  @Override
  public void init_loop() {
    input.update();
    telemetry.addData("Selected", servoSelector.selected());
    telemetry.addLine("Press X to select next");
  }

  @Override
  public void start() {
    servo = hardwareMap.servo.get(servoSelector.selected());
    input.clearRegistry();

    input.register(() -> gamepad1.dpad_up, () -> servo.setDirection(Servo.Direction.FORWARD))
        .register(() -> gamepad1.dpad_down, () -> servo.setDirection(Servo.Direction.REVERSE))
        .register(() -> gamepad1.y, () -> servo.setPosition(servo.getPosition() + POS_DELTA))
        .register(() -> gamepad1.a, () -> servo.setPosition(servo.getPosition() - POS_DELTA));
  }

  @Override
  public void loop() {
    telemetry.addLine("Controls")
        .addData("Dpad Up", "Sets direction to FORWARD")
        .addData("Dpad Down", "Sets direction to REVERSE")
        .addData("Y", "Increments position by %.3f", POS_DELTA)
        .addData("A", "Decrements position by %.3f", POS_DELTA);

    telemetry.addLine("Servo Data")
        .addData("Direction", servo.getDirection().name())
        .addData("Position", "%.4f", servo.getPosition());

    input.update();
  }
}
