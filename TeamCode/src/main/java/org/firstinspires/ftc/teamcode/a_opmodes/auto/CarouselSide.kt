package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
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

class CarouselSide(val opMode: LinearOpMode) {//TODO: possibly add the TeleOpPaths functionality to this

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

    private fun turn(from: Double, to: Double): AutoPathElement.Action {
        return AutoPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
                "to ${Math.toDegrees(to).roundToInt()}deg") {
            bot.roadRunner.turn(to - from)
        }
    }





    fun p2d(x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x,
            y,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h+PI
        )
    }

    fun p2D(x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x+8,
            y-7,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h-(PI/2)
        )
    }

    fun p2DwMore(x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -55.0,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else -65.0,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h-(PI/2)

//        if (GlobalConfig.alliance == GlobalConfig.Alliance.RED)
//            return Pose2d()
//        else if (GlobalConfig.alliance == GlobalConfig.Alliance.BLUE)
//            return Pose2d()
//        else
//            return Pose2d(z)
        )
    }

    fun p2dToHub(x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else y,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h+PI
        )}

    fun p2dSame (x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x,
            y,
            h
        )}

    fun v2D(v: Vector2d, h: Double): Pose2d{
        return Pose2d(if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) v.x else -v.x, v.y, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h-(PI/2))
    }

    fun v2D(x: Double, y: Double): Vector2d {
        return Vector2d(if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x+5.0, y)
    }




    val liftArm = CarouselSide.AutoPathElement.Action("Run outtake motor") {
        Thread.sleep(1000)
        bot.outtake.stopArm()
    }


    val runToHigh = CarouselSide.AutoPathElement.Action("Run outtake motor for high level") {
        bot.outtake.runToHigh();
    }

    val runToMid = CarouselSide.AutoPathElement.Action("Run outtake motor for middle level") {
        bot.outtake.runToMid();
    }

    val runToLow = CarouselSide.AutoPathElement.Action("Run outtake motor for low level") {
        bot.outtake.runToLow();
    }

    val restArm = CarouselSide.AutoPathElement.Action("outake to rest position"){
        bot.outtake.restArm();
    }


    val runIntake = CarouselSide.AutoPathElement.Action("Run intake") {
        bot.intake.spit();
    }

    val stopIntake = CarouselSide.AutoPathElement.Action("stop intake") {
        bot.intake.stop();
    }

    val flip = CarouselSide.AutoPathElement.Action("flip bucket"){
        bot.intake.flipBucket();
    }

    val unflip = CarouselSide.AutoPathElement.Action ("unflip bucket"){
        bot.intake.unflipBucket();
    }


    val clamp = CarouselSide.AutoPathElement.Action("Clamp claw") {
        bot.outtake.clamp();
    }

    val open = CarouselSide.AutoPathElement.Action("Clamp open") {
        bot.outtake.open();
    }


    //TODO: insert action vals here

    val runCarousel = CarouselSide.AutoPathElement.Action("Run carousel motor") {
        bot.carousel.run()
        Thread.sleep(3500)
        bot.carousel.stop()
    }


    fun edgeIn (): CarouselSide.AutoPathElement.Path{
        return CarouselSide.AutoPathElement.Path ("edge", bot.roadRunner.trajectoryBuilder(p2d(40.0, -20.0, 0.0))
            .lineToSplineHeading(p2dSame(65.5, 0.0, PI / 2))
            .addTemporalMarker(0.01, clamp.runner)
            .addTemporalMarker(0.2, restArm.runner)
            .addTemporalMarker(0.2,flip.runner)
            .build())
    }



    // go inside the warehouse while.... run intake, unflip, stopintake, flip
    fun inside (): CarouselSide.AutoPathElement.Path{
        return CarouselSide.AutoPathElement.Path ("Into Warehouse", bot.roadRunner.trajectoryBuilder(p2dSame(65.5, 0.0, PI / 2))
            .forward(48.0)
            .addTemporalMarker(0.01, runIntake.runner)
            .addTemporalMarker(0.01, unflip.runner)
            .build())
    }

    fun out (): CarouselSide.AutoPathElement.Path{
        return CarouselSide.AutoPathElement.Path ("Out of Warehouse", bot.roadRunner.trajectoryBuilder(p2dSame(65.5, 40.0, PI / 2))
            .back(48.0)
            .addTemporalMarker (0.1, flip.runner)
            .addTemporalMarker(0.3, runIntake.runner)
            .build())
    }

    fun startShippingHub (): CarouselSide.AutoPathElement.Path{
        return CarouselSide.AutoPathElement.Path ("Going to Shipping Hub", bot.roadRunner.trajectoryBuilder(p2D(63.0, -65.0, -PI / 2))
            .lineToSplineHeading(p2dToHub(40.0, -15.0, 0.0))
            .addTemporalMarker(0.03, runToHigh.runner)
            .build()
        )
    }





    fun carousel (): CarouselSide.AutoPathElement.Path{
        return CarouselSide.AutoPathElement.Path ("carousel", bot.roadRunner.trajectoryBuilder(p2d(65.5, -45.0, -PI / 2))
            .lineToSplineHeading(p2D(63.0, -65.0, -PI / 2))
            .build()
        )
    }



    fun parkWarehouse(): CarouselSide.AutoPathElement.Path{
        return CarouselSide.AutoPathElement.Path ("park in Warehouse", bot.roadRunner.trajectoryBuilder(p2dSame(65.5, 0.0, PI / 2))
            .addTemporalMarker (0.01, clamp.runner)
            .addTemporalMarker(0.1, restArm.runner)
            .forward (40.0)
            .build()
        )
    }


    fun parkColor () :CarouselSide.AutoPathElement.Path {
        return CarouselSide.AutoPathElement.Path(
            "park in colored square thing",
            bot.roadRunner.trajectoryBuilder(p2d(40.0, -25.0, 0.0))
                .splineToLinearHeading(Pose2d(v2D(34.0, -70.0), -PI / 2), -PI / 2)
                .build()
        )
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



    val startPose = p2d(  65.5, -36.0, -PI/2)

    //TODO: Make Trajectories in trajectorySets

    //                                                                              ====================================================
    private val trajectorySets: Map<TemplateDetector.PipelineResult, List<Any>> = mapOf(
        //use !! when accessing maps ie: dropSecondWobble[0]!!
        //example
        //
        TemplateDetector.PipelineResult.LEFT to run {
            listOf(
                clamp,
                carousel(),
                runCarousel,
                runToLow,
                startShippingHub(),
                open,
                parkColor(),
                clamp,
                restArm
//                clamp,
//                runToLow,
//                makePath("move to shipping hub",
//                    drive.trajectoryBuilder(p2d( 65.5, -36.0, -PI/2))
//                        .lineToSplineHeading(p2d(38.0, -13.0, 0.0)).build()),
//                open,
//                makePath("move to carousel",
//                    drive.trajectoryBuilder(p2d(38.0, -13.0, 0.0))
//                        .lineToSplineHeading(p2D(61.0,-60.0,-PI/2)).build()),
//                runCarousel,
//                makePath("move to red parking",
//                    drive.trajectoryBuilder(p2D(61.0,-60.0,-PI/2))
//                        .lineToSplineHeading(p2D(34.0, -66.0,-PI/2)).build()),
//                clamp,
//                restArm
            )
        },
        TemplateDetector.PipelineResult.MIDDLE to run {
            listOf(
                clamp,
                carousel(),
                runCarousel,
                runToMid,
                startShippingHub(),
                open,

                parkColor(),
                clamp,
                restArm
            )
        },
        TemplateDetector.PipelineResult.RIGHT to run {
            listOf(
                clamp,
                carousel(),
                runCarousel,
                runToHigh,
                startShippingHub(),
                open,
                parkColor(),
                clamp,
                restArm
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
        return (trajectorySets[a] as List<AutoPathElement>?)!!
    }


}

