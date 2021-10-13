package org.firstinspires.ftc.teamcode.d_util.toolbox;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;
import java.util.stream.Stream;

@TeleOp(name = "TBX: Motor Diagnostic", group = "ARC Toolbox")
public class MotorDiagnostic extends OpMode {
  private static int TP_DELTA = 5;

  private Selector motorSelector;
  private InputColumnResponder input = new InputColumnResponderImpl();
  private DcMotor motor;

  @Override
  public void init() {
    motorSelector = new Selector(hardwareMap.dcMotor.entrySet().stream().map(Map.Entry::getKey));
    input.register(() -> gamepad1.x, motorSelector::selectNext);
  }

  @Override
  public void init_loop() {
    input.update();
    telemetry.addData("Selected motor", motorSelector.selected());
    telemetry.addLine("Press X to select next");
  }

  @Override
  public void start() {
    motor = hardwareMap.dcMotor.get(motorSelector.selected());
    input.clearRegistry();

    input.register(() -> gamepad1.a, modeSelector::selectNext)
        .register(() -> gamepad1.b, zpbSelector::selectNext)
        .register(() -> gamepad1.dpad_up, () -> motor.setTargetPosition(motor.getTargetPosition() + TP_DELTA))
        .register(() -> gamepad1.dpad_down, () -> motor.setTargetPosition(motor.getTargetPosition() - TP_DELTA));
  }

  private Selector modeSelector = new Selector(enumOrdinals(DcMotor.RunMode.values())),
      zpbSelector = new Selector(enumOrdinals(DcMotor.ZeroPowerBehavior.values()));

  @Override
  public void loop() {
    telemetry.addLine("Controls")
        .addData("Left stick Y", "Controls Power Value")
        .addData("A", "Switches modes")
        .addData("B", "Toggles zero power behavior")
        .addData("Dpad Up", "Increases target position")
        .addData("Dpad Down", "Decreases target position");

    telemetry.addLine("Motor Data")
        .addData("Type", motor.getMotorType().getName())
        .addData("Power", "%.3f", motor.getPower())
        .addData("Mode", motor.getMode().name())
        .addData("Z.P.B.", motor.getZeroPowerBehavior().name())
        .addData("Current Pos", "%d", motor.getCurrentPosition())
        .addData("Target Pos", "%d", motor.getTargetPosition())
        .addData("Busy?", motor.isBusy() ? "Yes" : "No");

    telemetry.addLine("Selections")
        .addData("Selected Mode", modeSelector.selected())
        .addData("Selected ZPB", zpbSelector.selected());

    input.update();
    motor.setPower(-gamepad1.left_stick_y);
  }

  private <T extends Enum> Stream<String> enumOrdinals(T[] values) {
    return Stream.of(values)
        .map(Enum::name);
  }
}
