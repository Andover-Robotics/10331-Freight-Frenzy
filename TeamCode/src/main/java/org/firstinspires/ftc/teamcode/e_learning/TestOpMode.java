package org.firstinspires.ftc.teamcode.e_learning;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.types.PathType;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.b_hardware.Bot;

public class TestOpMode extends OpMode {
    Bot bot = Bot.getInstance(this);
    Path testPath;
    @Override
    public void init() {
        Waypoint p1 = new StartWaypoint(new Pose2d());
        Waypoint p2 = new PointTurnWaypoint(new Pose2d(2, 2, new Rotation2d(2.0)), 2, 2, 2, 2,2 );
        testPath = new Path(p1, p2);
        testPath.init();
        testPath.setPathType(PathType.WAYPOINT_ORDERING_CONTROLLED);//default
        testPath.enableRetrace();//default
        //remember to use testPath.reset() between uses
    }

    @Override
    public void loop() {
//        testPath.followPath(bot.drive, bot.roadRunner.localizer);//TODO: make ftclib odometry

        //testing music
//        MediaPlayer.create(hardwareMap.appContext, R.raw.rickroll).start();
    }
}
