package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.roundToInt

class AutoPaths(val opMode: LinearOpMode) {//TODO: possibly add the TeleOpPaths functionality to this

    //TODO: reverse this

    sealed class AutoPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory): AutoPathElement(name)
        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit): AutoPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    val Double.toRadians get() = (toRadians(this))
    val Int.toRadians get() = (this.toDouble().toRadians)
    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()

    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path{
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    //Probably won't be used, but here just in case
    fun makeAction(name: String, action: () -> Unit): AutoPathElement.Action{
        return AutoPathElement.Action(name, action)
        //Redundant but conforms to naming scheme
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit): MarkerCallback {
        override fun onMarkerReached() = func()
    }

    private fun turn(from: Double, to: Double): AutoPathElement.Action {
        return AutoPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
                "to ${Math.toDegrees(to).roundToInt()}deg") {
            bot.roadRunner.turn(to - from)
        }
    }


    //TODO: insert action vals here

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

    val startPose = Pose2d(0.0, 0.0, 0.0)

    //TODO: Make Trajectories in trajectorySets

    //                                                                              ====================================================
    private val trajectorySets: Map<TemplateDetector.PipelineResult, List<AutoPathElement>> = mapOf(
            //use !! when accessing maps ie: dropSecondWobble[0]!!
            //example
            TemplateDetector.PipelineResult.LEFT to run{
                listOf(
                makePath("forward 4",
                    drive.trajectoryBuilder(startPose)
                        .lineToConstantHeading(Vector2d(0.0, 12.0))
                        .build())
                )
            },
            TemplateDetector.PipelineResult.RIGHT to run{
                listOf(
                    makePath("forward 8",
                        drive.trajectoryBuilder(startPose)
                            .lineToConstantHeading(Vector2d(0.0, 24.0))
                            .build()),
                    makePath("left 8",
                        drive.trajectoryBuilder(lastPosition)
                            .lineToConstantHeading(Vector2d(24.0, 24.0))
                            .build()),
                    makePath("back 8",
                        drive.trajectoryBuilder(lastPosition)
                            .lineToConstantHeading(Vector2d(24.0, 0.0))
                            .build()),
                    makePath("right 8",
                        drive.trajectoryBuilder(lastPosition)
                            .lineToConstantHeading(startPose.vec())
                            .build()),
                    makeAction("wait for 3 seconds"){
                        Thread.sleep(3000)
                    },
                    makePath("spline forward",
                        drive.trajectoryBuilder(lastPosition)
                            .splineTo(Vector2d(24.0, 24.0), 0.0)
                            .build()),
                    makePath("spline backward",
                        drive.trajectoryBuilder(lastPosition, 180.0)
                            .splineTo(startPose.vec(), 180.0)
                            .build()),
                    makeAction("wait for 3 seconds"){
                        Thread.sleep(3000)
                    },
                    makePath("forward 8",
                        drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(Pose2d(0.0, 24.0, 90.toRadians))
                            .build()),
                    makePath("left 8",
                        drive.trajectoryBuilder(lastPosition)
                            .lineToLinearHeading(Pose2d(24.0, 24.0, PI))
                            .build()),
                    makePath("back 8",
                        drive.trajectoryBuilder(lastPosition)
                            .lineToLinearHeading(Pose2d(24.0, 0.0, -90.toRadians))
                            .build()),
                    makePath("right 8",
                        drive.trajectoryBuilder(lastPosition)
                            .lineToLinearHeading(startPose)
                            .build())

                )
            }
//
    )


    fun getTrajectories(a: TemplateDetector.PipelineResult): List<AutoPathElement>{
        return trajectorySets[a]!!
    }


}