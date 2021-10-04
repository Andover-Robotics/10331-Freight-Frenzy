package org.firstinspires.ftc.teamcode.hardware.oldSystemsForReference;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Intake extends SubsystemBase {
  public static double intakeSpeed = 1;

  public static class RunIntake extends CommandBase {
    private final Intake intake;

    public RunIntake(Intake intake) {
      this.intake = intake;      addRequirements(intake);
    }

    @Override
    public void initialize() {
      intake.run();
    }

    @Override
    public void end(boolean interrupted) {
      intake.stop();
    }
  }

//  public final Motor convBelt;
  private final Motor actuator;

  public Intake(OpMode opMode) {
//    convBelt = new Motor(opMode.hardwareMap, "convBelt", GoBILDA.RPM_1150);
    actuator = new Motor(opMode.hardwareMap, "intake", GoBILDA.RPM_1620);
  }

  public void run() {
    actuator.set(-intakeSpeed);
//    convBelt.set(conveyorSpeed);
  }

  public void run(double speed) {
    actuator.set(Math.abs(speed));
  }

  public void spit() {
    actuator.set(intakeSpeed);
//    convBelt.set(-conveyorSpeed);
  }

  public void stop() {
    actuator.stopMotor();
//    convBelt.stopMotor();
  }
}
