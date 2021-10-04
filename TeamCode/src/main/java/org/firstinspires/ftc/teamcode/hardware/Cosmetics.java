package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Cosmetics {//Literally a bunch of servos with stuff on them
  private Servo flag;

  public Cosmetics(OpMode opMode){
    flag = opMode.hardwareMap.servo.get("flag");
  }

  public void runFlag(){

  }
}
