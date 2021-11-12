package org.firstinspires.ftc.teamcode.b_hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive;

public class Bot {
  // in TeleOp and Autonomous we should be able to call "new Bot(this)"
  // bot.intake.run(), bot.shooter.spinUp
  public static Bot instance;


  //TODO: Declare subsystems here
  //example
  public final Carousel carousel;
  public final Intake intake;
//  public final TemplateSubsystem templateSubsystem;


  //required subsystems
  public final MecanumDrive drive;
  public final RRMecanumDrive roadRunner;
  public final BNO055IMU imu;
//  public final Cosmetics cosmetics;
//  public Pair<ExpansionHubEx, ExpansionHubEx> hubs = null;
  public OpMode opMode;

  /** Get the current Bot instance from somewhere other than an OpMode */
  public static Bot getInstance() {
    if (instance == null) {
      throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
    }
    return instance;
  }

  public static Bot getInstance(OpMode opMode) {
    if (instance == null) {
      return instance = new Bot(opMode);
    }
    instance.opMode = opMode;
    return instance;
  }

  public void reset(){
    //TODO: add reset code here
  }

  private Bot(OpMode opMode){
    this.opMode = opMode;
    enableAutoBulkRead();
    try {
//      this.hubs = Pair.create(opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"), // TODO: check if revextensions2 works with sdk7.0 and control hubs
//          opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"));
    } catch (Exception e) {
      // Avoid catastrophic errors if RevExtensions don't behave as expected. Limited trust of stability
      e.printStackTrace();
    }

    //TODO: initialize subsystems
    //example
//    this.templateSubsystem = new TemplateSubsystem(opMode);
    this.carousel = new Carousel(opMode);
    this.intake = new Intake(opMode);



    //required subsystems
    this.drive = new MecanumDrive(false,
        new MotorEx(opMode.hardwareMap, GlobalConfig.motorFL),
        new MotorEx(opMode.hardwareMap, GlobalConfig.motorFR),
        new MotorEx(opMode.hardwareMap, GlobalConfig.motorBL),
        new MotorEx(opMode.hardwareMap, GlobalConfig.motorBR));
    this.roadRunner = new RRMecanumDrive(opMode.hardwareMap);
//    this.cosmetics = new Cosmetics(opMode);
    imu = roadRunner.imu;
  }

//  private void initializeImu() {
//    final Parameters params = new Parameters();
//    params.angleUnit = AngleUnit.RADIANS;
//    imu.initialize(params);
//  }

  private void enableAutoBulkRead() {
    for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
      mod.setBulkCachingMode(BulkCachingMode.AUTO);
    }
  }
}
