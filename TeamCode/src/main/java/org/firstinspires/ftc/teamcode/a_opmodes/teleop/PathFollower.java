package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.a_opmodes.teleop.TeleOpPaths.TeleOpPathElement;
import org.firstinspires.ftc.teamcode.b_hardware.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.b_hardware.drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;
import org.firstinspires.ftc.teamcode.a_opmodes.teleop.MainTeleOp.TemplateState;

import java.util.Map;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class PathFollower {//May or may not be used depending on the game
  //TODO: INVESTIGATE PURE PURSUIT
  private Bot bot = Bot.getInstance();
  private RRMecanumDrive drive = bot.roadRunner;
  private OpMode opmode;
  private TeleOpPaths paths;

  public PathFollower(OpMode opmode){
    this.opmode = opmode;
    this.paths = new TeleOpPaths(this.opmode);
  }

  private void goToPose(Pose2d pose){
    if(Math.abs(drive.getPoseEstimate().getX() - pose.getX()) + Math.abs(drive.getPoseEstimate().getY() - pose.getY()) > 0.5 ||
      Math.abs(drive.getPoseEstimate().minus(pose).getHeading()) > 0.5)
      drive.followTrajectoryAsync(
          drive.trajectoryBuilder(drive.getPoseEstimate())
          .lineToLinearHeading(pose)
          .build());
  }

  private void followAction(TemplateState state, Function0<Unit> runner){
    runner.invoke();
  }


  public void followPath(TemplateState state, int percent, int part){
    TeleOpPathElement element = getElement(state, part);
    if(element instanceof TeleOpPathElement.Path){
      goToPose(((TeleOpPathElement.Path)element).getTrajPose(percent));
    }else if(element instanceof TeleOpPathElement.Action){
      followAction(state, ((TeleOpPathElement.Action)element).getRunner(percent));
    }else{
      goToPose(((TeleOpPathElement.ActionPath)element).getTrajPose(percent));
      followAction(state, ((TeleOpPathElement.ActionPath)element).getRunner(percent));
    }
  }

  public Pose2d getPose(TemplateState state, int percent, int part){
    TeleOpPathElement element = getElement(state, part);
    if(element instanceof TeleOpPathElement.Path){
      return ((TeleOpPathElement.Path) element).getTrajPose(percent);
    }else if(element instanceof TeleOpPathElement.ActionPath){
      return ((TeleOpPathElement.ActionPath) element).getTrajPose(percent);
    }
    return null;
  }

  public boolean isTrajectory(TemplateState state, int part){
    return getElement(state, part) instanceof TeleOpPathElement.Path || getElement(state, part) instanceof TeleOpPathElement.ActionPath;
  }

  private TeleOpPaths.TeleOpPathElement getElement(TemplateState state, int part){
    return paths.getTrajectory(state, part);
  }

  public Map<TemplateState, Integer> getPathsInfo(){
    return paths.getTrajectoryListInfo();
  }

  public void stop(){
    drive.mode = Mode.IDLE;
    drive.setDriveSignal(new DriveSignal());
    bot.reset();
  }
}
