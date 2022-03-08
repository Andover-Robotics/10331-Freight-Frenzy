package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

@TeleOp(name = "TeleOp with Field Centric", group = "Competition")
public class TeleOpWithCentric extends BaseOpMode {//required vars here
    private double cycle = 0;
    private double prevRead = 0;
    private TimingScheduler timingScheduler;
    private boolean centricity = false;
    private boolean isManual = true;
    private int percent = 1, part = 0;
    private final double slowModeSpeed = 0.4;
    // private boolean clawIsOpen = false;

    private final double SLOW_MODE_PERCENT = 0.4;
    private double fieldCentricOffset0 = 0.0;
    private double fieldCentricOffset1 = 0.0;



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


//  private MotorEx carousel;


    void subInit() {
        //TODO: initialize subsystems not initialized in bot constructor
        timingScheduler = new TimingScheduler(this);
//    carousel = new MotorEx(hardwareMap, "carousel");
//    carousel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//    carousel.set(0);
        // bot.outtake.clamp();

    }

    @Override
    public void subLoop() {
        //update stuff=================================================================================================
        cycle = 1.0 / (time - prevRead);
        prevRead = time;
        timingScheduler.run();
        long profileStart = System.currentTimeMillis();

        //Movement =================================================================================================
        //TODO: change depending on mode :)
        driveSpeed = 1 - 0.1 * (triggerSignal(Trigger.LEFT_TRIGGER) + triggerSignal(Trigger.RIGHT_TRIGGER));

        if (justPressed(Button.BACK)) {
            isManual = !isManual;
        }

        if (isManual) {
            drive();
        } else {
            followPath();
        }


        //TODO: insert actual teleop stuff here
//    if(buttonSignal(Button.DPAD_UP)){
//      bot.carousel.run();
//    }else{
//      bot.carousel.stop();
//    }
//




        //telemetry.addData("outtake down update", System.currentTimeMillis() - profileStart);




        if (gamepadEx2.getButton(Button.Y)) {
            bot.intake.run();
        }else if (gamepadEx2.getButton(Button.X)){
            bot.intake.spit();
        }else{
            bot.intake.stop();
        }



        if(gamepadEx2.getTrigger(Trigger.RIGHT_TRIGGER) > 0.01){
            bot.outtake.runArm(gamepadEx2.getTrigger(Trigger.RIGHT_TRIGGER));
        }else if(gamepadEx2.getTrigger(Trigger.LEFT_TRIGGER) > 0.01){
            bot.outtake.runArm(-gamepadEx2.getTrigger(Trigger.LEFT_TRIGGER));
        }else{
            bot.outtake.stopArm();
        }



        if(gamepadEx2.getButton(Button.DPAD_LEFT)) {
            bot.outtake.clamp();
        }




        if(gamepadEx2.getButton(Button.DPAD_RIGHT)){
            bot.outtake.open();
        }





        if(gamepadEx2.getButton(Button.DPAD_UP)){
            bot.carousel.run();
        }else{
            bot.carousel.stop();
        }



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
    A: toggle claw    B: right intake       X: reverse left and right intake     Y: left intake
    DPAD
    L:      D:     U: carousel      R:
    Joystick
    L:
    R:
    Trigger
    L: outtake arm thing up (counter clockwise) R:move outtake arm thing down (clockwise)
    Bumper:
    L:move claw for left intake  R: move claw for right intake

    Other
    Start:  Back:switch between automation and driving
     */


    /*
    AUTOMATION CONTROL SCHEME

     */



        CommandScheduler.getInstance().run();

        // TODO organize this test code
        updateLocalization();

        telemetry.addData("telemetry things update", System.currentTimeMillis() - profileStart);

        telemetry.addData("percent", percent);
        telemetry.addData("part", part);
        telemetry.addData("cycle", cycle);
        telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
        telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
        telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
        telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);
    }


    private void drive() {//Driving ===================================================================================
        final double gyroTolerance = 0.05;

        final double gyroAngle0 =
                bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                        - fieldCentricOffset0;
        final double gyroAngle1 = (bot.imu1 != null) ?
                bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                        - fieldCentricOffset1
                : gyroAngle0;
        final double avgGyroAngle = ((gyroAngle0 + gyroAngle1) / 2);

        Vector2d driveVector = new Vector2d(gamepadEx1.getLeftX(), gamepadEx1.getLeftY()),
                turnVector = new Vector2d(
                        gamepadEx1.getRightX() * Math.abs(gamepadEx1.getRightX()),
                        0);
        if (bot.roadRunner.mode == RRMecanumDrive.Mode.IDLE) {

            boolean dpadPressed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    || gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT));
            boolean buttonPressed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B));

            double forwardSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : -1) : 0;
            double strafeSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : -1) : 0;
            double turnSpeed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B)) ? (gamepadEx1.getButton(GamepadKeys.Button.B) ? 1 : -1) : 0;

            if (centricity) {//epic java syntax
                bot.drive.driveFieldCentric(
                        driveVector.getY() * driveSpeed,
                        driveVector.getX() * -driveSpeed,
                        turnVector.getX() * driveSpeed,
                        (Math.abs(avgGyroAngle - gyroAngle0) < gyroTolerance
                                || Math.abs(avgGyroAngle - gyroAngle1) < gyroTolerance) ?
                                Math.abs(gyroAngle0 - avgGyroAngle) <
                                        Math.abs(gyroAngle1 - avgGyroAngle) ?
                                        gyroAngle0 : gyroAngle1 : avgGyroAngle
                        // Epic Java Syntax here
                        /*
                         * In theory, this check ensures that when the avgGyroAngle is VERY off
                         * due to one IMU giving ~0.01, and the second giving ~1.99 which SHOULD be considered an angle of 2 or 0
                         * This problem was encountered while first testing the dual IMU dependant field centric drive
                         * the robot would run two motors on the corners of the robot in opposite directions, causing negligible movement
                         * Because I believe the rarer incorrect averages, these ternary statements, should correct this.
                         */
                );
            } else if (dpadPressed || buttonPressed) {
                double tempDriveSpeed = driveSpeed *= SLOW_MODE_PERCENT;
                bot.drive.driveRobotCentric(
                        strafeSpeed * tempDriveSpeed,
                        forwardSpeed * -tempDriveSpeed,
                        turnSpeed * tempDriveSpeed
                );
            } else {
                bot.drive.driveRobotCentric(
                        driveVector.getY() * driveSpeed,
                        driveVector.getX() * -driveSpeed,
                        turnVector.getX() * driveSpeed
                );
            }

        }
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            fieldCentricOffset0 = bot.imu0.getAngularOrientation()
                    .toAngleUnit(AngleUnit.DEGREES).firstAngle;
            fieldCentricOffset1 = bot.imu1.getAngularOrientation()
                    .toAngleUnit(AngleUnit.DEGREES).firstAngle;
        }
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
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
