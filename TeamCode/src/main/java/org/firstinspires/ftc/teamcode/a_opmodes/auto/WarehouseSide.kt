package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector
import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.roundToInt

class WarehouseSide(val opMode: LinearOpMode) {//TODO: possibly add the TeleOpPaths functionality to this

    //TODO: reverse this


    sealed class AutoPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory) : AutoPathElement(name)

        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit) : AutoPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    val Double.toRadians get() = (toRadians(this))
    val Int.toRadians get() = (this.toDouble().toRadians)
    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()

    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path {
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    //Probably won't be used, but here just in case
    fun makeAction(name: String, action: () -> Unit): AutoPathElement.Action {
        return AutoPathElement.Action(name, action)
        //Redundant but conforms to naming scheme
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit) : MarkerCallback {
        override fun onMarkerReached() = func()
    }

    fun p2d(x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x,
            y,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else -h
        )
    }

    private fun turn(from: Double, to: Double): AutoPathElement.Action {
        return AutoPathElement.Action(
            "Turn from ${Math.toDegrees(from).roundToInt()}deg" +
                    "to ${Math.toDegrees(to).roundToInt()}deg"
        ) {
            bot.roadRunner.turn(to - from)
        }
    }


    private val liftArm = AutoPathElement.Action("Run outtake motor") {
        Thread.sleep(1000)
        bot.outtake.stopArm()
    }

    private val runToHigh = AutoPathElement.Action("Run outtake motor for high level") {
        bot.outtake.runToHigh();
    }

    private val runToMid = AutoPathElement.Action("Run outtake motor for middle level") {
        bot.outtake.runToMid();
    }

    private val runToLow = AutoPathElement.Action("Run outtake motor for low level") {
        bot.outtake.runToLow();
    }

    private val clamp = AutoPathElement.Action("Clamp claw") {
        bot.outtake.clamp();
    }

    private val open = AutoPathElement.Action("Clamp open") {
        bot.outtake.open();
    }


    //TODO: insert action vals here

    private val runCarousel = AutoPathElement.Action("Run carousel motor") {
        bot.carousel.run()
        Thread.sleep(5000)
        bot.carousel.stop()
    }

    private val downArm = AutoPathElement.Action("Run outtake motor to rest") {
        bot.outtake.restArm();
    }

    private val runIntake = AutoPathElement.Action ("Run Intake") {
        bot.intake.runLeft();
        Thread.sleep(2000);
        bot.intake.stop();
    }


    //                                                                  =======================================================

    //example
    //private val shootRings = AutoPathElement.Action("Shoot 3 rings") {
    //        bot.shooter.shootRings(opMode, 3, 0.8)
    //        bot.shooter.turnOff()
    //        Thread.sleep(1000)
    //    }


    //Copy from here on into AutoPathVisualizer ==============================================================================

    //TODO: Insert pose/vector vals here


    //                                                                  ===================================================

    //example
    // private val dropSecondWobble = mapOf(
    //            0 to Pose2d(-4.2 + 1.5, -48.0 - 3.056 + 1f, (-90.0 + 30.268).toRadians),
    //            1 to Pose2d(24.0 - 9.45428 + 3f, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
    //            4 to Pose2d(48 - 5.1, -48.0 - 3.0556 - 3f, (-90.0 + 30.268).toRadians)
    //    )


    val startPose = p2d(65.5, 12.0, -PI / 2)

    //TODO: Make Trajectories in trajectorySets

    //                                                                              ====================================================
    private val trajectorySets: Map<TemplateDetector.PipelineResult, List<AutoPathElement>> = mapOf(
        //use !! when accessing maps ie: dropSecondWobble[0]!!
        //example
        //
        TemplateDetector.PipelineResult.LEFT to run {
            listOf(
                clamp,
                runToLow,
                makePath(
                    "move to shipping hub",
                    drive.trajectoryBuilder(Pose2d(65.5, 12.0, -PI / 2))
                        .lineToSplineHeading(Pose2d(38.0, -10.0, 0.0)).build()
                ),
                open,
                makePath(
                    "move to warehouse",
                    drive.trajectoryBuilder(p2d(63.0, -61.0, -PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 36.0, PI / 2)).build()
                ),
                open,
                runIntake,
                clamp,
                makePath(
                    "move to shipping hub",
                    drive.trajectoryBuilder(p2d(65.5, 36.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(38.0, -10.0, 0.0)).build()
                ),
                runToHigh,
                open,
                clamp,
                downArm,
                makePath(
                    "move to warehouse",
                    drive.trajectoryBuilder(p2d(63.0, -61.0, -PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 36.0, PI / 2)).build()
                ),
                open,
                runIntake,
                clamp,
                makePath(
                    "move to shipping hub",
                    drive.trajectoryBuilder(p2d(65.5, 36.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(38.0, -10.0, 0.0)).build()
                ),
                runToHigh,
                open,
                clamp,
                downArm,
                makePath(
                    "move to warehouse parking",
                    drive.trajectoryBuilder(p2d(38.0,-10.0,0.0))
                        .lineToSplineHeading(p2d(65.5,12.0,PI/2))
                        .lineToSplineHeading(p2d(65.5,38.0,PI/2)).build()
                )
            )
        },
        TemplateDetector.PipelineResult.MIDDLE to run {
            listOf(
                clamp,
                runToMid,
                makePath(
                    "move to shipping hub",
                    drive.trajectoryBuilder(Pose2d(65.5, 12.0, -PI / 2))
                        .lineToSplineHeading(Pose2d(38.0, -10.0, 0.0)).build()
                ),
                open,
                makePath(
                    "move to warehouse",
                    drive.trajectoryBuilder(p2d(63.0, -61.0, -PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 36.0, PI / 2)).build()
                ),
                open,
                runIntake,
                clamp,
                makePath(
                    "move to shipping hub",
                    drive.trajectoryBuilder(p2d(65.5, 36.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(38.0, -10.0, 0.0)).build()
                ),
                runToHigh,
                open,
                clamp,
                downArm,
                makePath(
                    "move to warehouse",
                    drive.trajectoryBuilder(p2d(63.0, -61.0, -PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 36.0, PI / 2)).build()
                ),
                open,
                runIntake,
                clamp,
                makePath(
                    "move to shipping hub",
                    drive.trajectoryBuilder(p2d(65.5, 36.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(38.0, -10.0, 0.0)).build()
                ),
                runToHigh,
                open,
                clamp,
                downArm,
                makePath(
                    "move to warehouse parking",
                    drive.trajectoryBuilder(p2d(38.0,-10.0,0.0))
                        .lineToSplineHeading(p2d(65.5,12.0,PI/2))
                        .lineToSplineHeading(p2d(65.5,38.0,PI/2)).build()
                )
            )
        },
        TemplateDetector.PipelineResult.RIGHT to run {
            listOf(
                clamp,
                runToHigh,
                makePath(
                    "move to shipping hub",
                    drive.trajectoryBuilder(Pose2d(65.5, 12.0, -PI / 2))
                        .lineToSplineHeading(Pose2d(38.0, -10.0, 0.0)).build()
                ),
                open,
                makePath(
                    "move to warehouse",
                    drive.trajectoryBuilder(p2d(63.0, -61.0, -PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 36.0, PI / 2)).build()
                ),
                open,
                runIntake,
                clamp,
                makePath(
                    "move to shipping hub",
                    drive.trajectoryBuilder(p2d(65.5, 36.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(38.0, -10.0, 0.0)).build()
                ),
                runToHigh,
                open,
                clamp,
                downArm,
                makePath(
                    "move to warehouse",
                    drive.trajectoryBuilder(p2d(63.0, -61.0, -PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 36.0, PI / 2)).build()
                ),
                open,
                runIntake,
                clamp,
                makePath(
                    "move to shipping hub",
                    drive.trajectoryBuilder(p2d(65.5, 36.0, PI / 2))
                        .lineToSplineHeading(p2d(65.5, 12.0, PI / 2))
                        .lineToSplineHeading(p2d(38.0, -10.0, 0.0)).build()
                ),
                runToHigh,
                open,
                clamp,
                downArm,
                makePath(
                    "move to warehouse parking",
                    drive.trajectoryBuilder(p2d(38.0,-10.0,0.0))
                        .lineToSplineHeading(p2d(65.5,12.0,PI/2))
                        .lineToSplineHeading(p2d(65.5,38.0,PI/2)).build()
                )

            )
        }
//
    )


    fun park(result: TemplateDetector.PipelineResult): List<AutoPathElement> {
        return run {
            listOf(
                makePath(
                    "drive into warehouse",
                    drive.trajectoryBuilder(startPose)
                        .forward(72.0)
                        .build()
                )
            )
        }
    }


    fun getTrajectories(a: TemplateDetector.PipelineResult): List<AutoPathElement> {
        return trajectorySets[a]!!
    }


}