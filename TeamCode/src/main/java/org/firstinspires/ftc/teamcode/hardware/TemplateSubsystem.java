package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.Motor.RunMode;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TemplateSubsystem extends SubsystemBase {
  //Put final vars in GlobalConfig
  public static int slideStage = 0;
  public static final int upPosition = 1;
  private MotorEx motorEx;
  public TemplateSubsystem(OpMode opMode){
    motorEx = new MotorEx(opMode.hardwareMap, "templateMotorEx", GoBILDA.RPM_312);
    motorEx.setRunMode(RunMode.PositionControl);
  }

  public void operateSlides(double stage){
    motorEx.setTargetPosition((int) (stage * 4));
  }

}
