package org.firstinspires.ftc.teamcode.a_opmodes.auto


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.*
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector
import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.roundToInt



class FullSide(val opMode: LinearOpMode)  {//TODO: possibly add the TeleOpPaths functionality to this

    //TODO: reverse this


    sealed class AutoPathElement(open val name: String) {
        //class AutoPaths(val opMode: LinearOpMode) {
        class Path(override val name: String, val trajectory: Trajectory) : AutoPathElement(name)

        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit) : AutoPathElement(name)
        // Command
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
        return AutoPathElement.Action(
            "Turn from ${Math.toDegrees(from).roundToInt()}deg" +
                    "to ${Math.toDegrees(to).roundToInt()}deg"
        ) {
            bot.roadRunner.turn(to - from)
        }
    }


    fun p2d(x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x,
            y,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h + PI
        )
    }

    fun p2D(x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x,
            y,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h - (PI / 2)
        )
    }

    fun p2DwMore(x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -55.0,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else -64.0,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h - (PI / 2)

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
            y,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h+PI
        )}


    fun p2dSame (x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x,
            y,
            h
        )}

    class FollowTrajectory(val bot: Bot, val trajectory: Trajectory) : CommandBase() {
        override fun initialize() = bot.roadRunner.followTrajectoryAsync(trajectory)
        override fun execute() = bot.roadRunner.update()
        override fun isFinished() = !bot.roadRunner.isBusy
    }

    fun v2D(v: Vector2d, h: Double): Pose2d {
        return Pose2d(
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) v.x else -v.x,
            v.y,
            if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h - (PI / 2)
        )
    }

    fun v2D(x: Double, y: Double): Vector2d {
        return Vector2d(if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else -x + 5.0, y)
    }




    val liftArm = AutoPathElement.Action("Run outtake motor") {
        Thread.sleep(1000)
        bot.outtake.stopArm()
    }


    val runToHigh = AutoPathElement.Action("Run outtake motor for high level") {
        bot.outtake.runToHigh();
    }

    val runToMid = AutoPathElement.Action("Run outtake motor for middle level") {
        bot.outtake.runToMid();
    }

    val runToLow = AutoPathElement.Action("Run outtake motor for low level") {
        bot.outtake.runToLow();
    }

    val restArm = AutoPathElement.Action("outake to rest position"){
        bot.outtake.restArm();
    }


    val runIntake = AutoPathElement.Action("Run intake") {
        bot.intake.run();
    }

    val stopIntake = AutoPathElement.Action("stop intake") {
        bot.intake.stop();
    }

    val flip = AutoPathElement.Action("flip bucket"){
        bot.intake.flipBucket();
    }

    val unflip = AutoPathElement.Action ("unflip bucket"){
        bot.intake.unflipBucket();
    }


    val clamp = AutoPathElement.Action("Clamp claw") {
        bot.outtake.clamp();
    }

    val open = AutoPathElement.Action("Clamp open") {
        bot.outtake.open();
    }


    //TODO: insert action vals here

    val runCarousel = AutoPathElement.Action("Run carousel motor") {
        bot.carousel.run()
        Thread.sleep(2500)
        bot.carousel.stop()
    }




// to go edge while... clamp, restArm, flip,

    fun edgeIn (): AutoPathElement.Path{
        return AutoPathElement.Path ("edge", bot.roadRunner.trajectoryBuilder(p2d(45.0, -15.0, 0.0))
            .lineToSplineHeading(p2dSame(65.5, 0.0, PI / 2))
            .addTemporalMarker(0.01, clamp.runner)
            .addTemporalMarker(0.2, restArm.runner)
            .addTemporalMarker(0.2,flip.runner)
            .build())
    }


    fun edgeInCarousel (): AutoPathElement.Path{
        return AutoPathElement.Path ("edge", bot.roadRunner.trajectoryBuilder(p2D(63.0, -70.0, -PI / 2))
            //.lineToSplineHeading(p2dSame(65.5, 0.0, PI / 2))
            .strafeLeft(60.0)
            .addTemporalMarker(0.01, clamp.runner)
            .addTemporalMarker(0.2, restArm.runner)
            .addTemporalMarker(0.2,flip.runner)
            .build())
    }


    // go inside the warehouse while.... run intake, unflip, stopintake, flip
    fun inside (): AutoPathElement.Path{
        return AutoPathElement.Path ("Into Warehouse", bot.roadRunner.trajectoryBuilder(p2dSame(65.5, 0.0, PI / 2))
            .forward(48.0)
            .addTemporalMarker(0.01, runIntake.runner)
            .addTemporalMarker(0.01, unflip.runner)
            .build())
    }

    fun out (): AutoPathElement.Path{
        return AutoPathElement.Path ("Out of Warehouse", bot.roadRunner.trajectoryBuilder(p2dSame(65.5, 40.0, PI / 2))
            .back(48.0)
            .addTemporalMarker(0.1, open.runner)
            .addTemporalMarker (0.1, flip.runner)
            .addTemporalMarker(0.3, runIntake.runner)
            .build())
    }

    fun shippingHub (): AutoPathElement.Path{
        return AutoPathElement.Path ("Going to Shipping Hub", bot.roadRunner.trajectoryBuilder(p2dSame(65.5, -8.0, PI / 2))
            .lineToSplineHeading(p2d(45.0, -15.0, 0.0))
            .addTemporalMarker(0.01, clamp.runner)
            .addTemporalMarker(0.03, runToHigh.runner)
            .build()
        )
    }

    fun carousel (): AutoPathElement.Path{
        return AutoPathElement.Path ("carousel", bot.roadRunner.trajectoryBuilder(p2d(45.0, -15.0, 0.0))
            .lineToSplineHeading(p2D(63.0, -75.0, -PI / 2))
            .build()
        )
    }

    fun parkWarehouse(): AutoPathElement.Path{
        return AutoPathElement.Path ("park in Warehouse", bot.roadRunner.trajectoryBuilder(p2dSame(65.5, 0.0, PI / 2))
            .addTemporalMarker (0.01, clamp.runner)
            .addTemporalMarker(0.1, restArm.runner)
            .forward (40.0)
            .build()
        )
    }


    fun startShippingHub (): FullSide.AutoPathElement.Path{
        return FullSide.AutoPathElement.Path ("Going to Shipping Hub", bot.roadRunner.trajectoryBuilder(p2d(65.5, -40.0, -PI / 2))
            .lineToSplineHeading(p2dToHub(40.0, -15.0, 0.0))
            .addTemporalMarker(0.03, runToHigh.runner)
            .build()
        )
    }



//    val downArm = AutoPathElement.Action("Run outtake motor to rest") {
//        bot.outtake.restArm();
//    }


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


    val startPose = p2d(65.5, -45.0, -PI / 2)

    //TODO: Make Trajectories in trajectorySets

    //
    //
    //                                                                        ====================================================
//    val moveToCaroselAndLowerArm = AutoPathElement.Action("move to carosel while lowering arm :)") {
//        ParellelCommandGroup(
//                makePath(
//            "move to edge (hub to warehouse)",
//            drive.trajectoryBuilder(p2dToHub(63.0,-61.0,0.0))
//                .lineToSplineHeading(p2dSame(65.5,  0.0,PI/2)).build()
//        ),
//        clamp,
//        downArm).
//    }

    val trajectorySets: Map<TemplateDetector.PipelineResult, List<Any>> = mapOf(
        TemplateDetector.PipelineResult.LEFT to run {
            listOf(
            )
        },
        TemplateDetector.PipelineResult.MIDDLE to run {
            listOf(
           )
        },
        TemplateDetector.PipelineResult.RIGHT to run {
            listOf(//add intake when inside wrehouse and open claw when awt shipping hub '
                clamp,
                runToHigh,
                startShippingHub(),
                open,
                carousel(),
                runCarousel,
                edgeInCarousel(),
                inside(),
                stopIntake,
                flip,
                out(),
                stopIntake,
                shippingHub(),
                open,
                edgeIn(),
                inside(),
                stopIntake,
                flip,
                out(),
                stopIntake,
                shippingHub(),
                open,
                edgeIn(),
                parkWarehouse()
            )


//            listOf(
//                clamp,
//                makePath(
//                    "move to carousel",
//                    drive.trajectoryBuilder(p2d(65.5, -45.0, -PI / 2))
//                        .lineToSplineHeading(p2D(63.0, -61.0, -PI / 2)).build()
//                ),
//                runCarousel,
//                runToHigh,
//                makePath(
//                    "move to shipping hub",
//                    drive.trajectoryBuilder(p2d(63.0, -61.0, -PI / 2))
//                        .lineToSplineHeading(p2d(40.0, -8.0, 0.0)).build()
//                ),
//                    makePath(
//                    "move to edge (hub to warehouse)",
//                        drive.trajectoryBuilder(p2dToHub(63.0,-61.0,0.0))
//                            .lineToSplineHeading(p2dSame(65.5, 0.0,PI/2)).build()
//                    ),
//                clamp,
//                downArm,
//                makePath(
//                    "move into warehouse",
//                    drive.trajectoryBuilder(p2dSame(65.5, 0.0, PI/2))
//                        .forward(48.0).build()
//                ),
//                open,
//                runIntake,
//                clamp,
//                runToHigh,
//                makePath(
//                    "move to edge (warehouse to hub)",
//                    drive.trajectoryBuilder(p2dSame(65.5, 48.0, PI/2))
//                        .back(48.0).build()
//                ),
//
//                makePath(
//                    "move to shipping hub",
//                    drive.trajectoryBuilder(p2dSame(65.5, 0.0, PI/2))
//                        .lineToSplineHeading(p2dToHub(40.0,-15.0,0.0)).build()
//                ),
//                open,
//                makePath(
//                    "move to edge (hub to warehouse)",
//                    drive.trajectoryBuilder(p2dToHub(40.0,-15.0,0.0))
//                        .lineToSplineHeading(p2dSame(65.5, 0.0, PI/2)).build()
//                ),
//                clamp,
//                makePath(
//                    "move to warehouse parking",
//                    drive.trajectoryBuilder(p2dSame(65.5,0.0,PI/2))
//                        .forward(48.0).build()
//                ),
//                downArm
//
//            )
        }
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