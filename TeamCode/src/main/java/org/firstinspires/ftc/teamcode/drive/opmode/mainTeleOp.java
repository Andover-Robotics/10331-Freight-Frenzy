/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.drive.opmode;
import com.arcrobotics.ftclib.drivebase.*;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import  com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//commenting things
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * ooga booga
 */

//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tank Drive TeleOp", group = "Iterative Opmode")
//@Disabled
//@TeleOp(name = "Mecanum Drive Train")
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mecanum Drive TeleOp", group = "Iterative Opmode")
public class mainTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    //NEW FLYWHEELS
    //private DcMotor intakeFlywheel, outtakeFlywheel;
    // boolean to see if Flywheels are running
    //private boolean runIntakeForward = false, runOuttakeForward = false, stop = false, runServosForward = false, runServosBackward = false;
    //Using ARC-Core's Mecanum Drive class, we initialized a Mecanum Drive as seen below
    MecanumDrive mecanumDrive; // this is the object that we will be using to control the mecanum drive

    // New servo to move foundation
    //private Servo servo;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFrontDrive = new MotorEx(hardwareMap, "leftFrontDrive",Motor.GoBILDA.RPM_312);
        leftBackDrive = new MotorEx(hardwareMap, "leftBackDrive",Motor.GoBILDA.RPM_312);
        rightFrontDrive = new MotorEx(hardwareMap, "rightFrontDrive",Motor.GoBILDA.RPM_312);
        rightBackDrive = new MotorEx(hardwareMap, "rightBackDrive",Motor.GoBILDA.RPM_312);

        //intakeFlywheel = hardwareMap.get(DcMotor.class, "intakeFlywheel");
        //outtakeFlywheel = hardwareMap.get(DcMotor.class, "outtakeFlywheel");

        // servo = hardwareMap.get(Servo.class, "servo");

        // Initialize our Mecanum Drive using our motors as parameters
        // Set default power of mecanum drive to 1.0
        //iveTrain = MecanumDrive.fromCrossedMotors(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, this, 89, 1120);;
        mecanumDrive = new MecanumDrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive); //figure out with michael how to properly use the mecanum drive stuff
        //mecanumDrive.setRunMode(Motor.RunMode.PositionControl);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setRunMode(MotorEx.RunMode.RawPower); // setting direction for REGULAR motors?
        // what is position control (ask michael!)
        leftBackDrive.setRunMode(MotorEx.RunMode.RawPower);
        rightFrontDrive.setRunMode(MotorEx.RunMode.RawPower);
        rightBackDrive.setRunMode(MotorEx.RunMode.RawPower);

        /*leftFrontDrive.setDirection(Motor.Direction.FORWARD); // setting direction for REGULAR motors?
        leftBackDrive.setDirection(Motor.Direction.FORWARD);
        rightFrontDrive.setDirection(Motor.Direction.REVERSE);
        rightBackDrive.setDirection(Motor.Direction.REVERSE);*/

        // Flywheels move backwards to move blocks inward
        //intakeFlywheel.setDirection(DcMotor.Direction.REVERSE);
        //outtakeFlywheel.setDirection(DcMotor.Direction.REVERSE);

        /*leftFrontFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        //intakeFlywheel.setPower(0); //since these are Dcmotors they can use the fucntion
        //outtakeFlywheel.setPower(0);


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    int count = 0;

    @Override
    public void loop() {
        count++;
        telemetry.addLine(((Integer) count).toString());
        //telemetry.addData("LF Flywheel Power: ", intakeFlywheel.getPower());

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y; // controls up and down movement
        //based off the assumption that these are giving numerical values between -1,0,1
        double strafe = gamepad1.left_stick_x; // the movement left to right of robot
        double turn = gamepad1.right_stick_x; //same with above


        leftFrontDrive.set(drive + strafe + turn);
        leftBackDrive.set(drive - strafe + turn);
        rightFrontDrive.set(drive - strafe - turn);
        rightBackDrive.set(drive + strafe - turn);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //test
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {

    }



    //test commit

}