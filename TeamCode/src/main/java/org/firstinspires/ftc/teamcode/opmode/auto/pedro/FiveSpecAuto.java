package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;

@Config
@Autonomous(group="production")
public class FiveSpecAuto extends LinearOpMode {
    private ElapsedTime timer;
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private DashboardPoseTracker dashboardPoseTracker;
    Robot robot;
    public static double testX = 27;
    public static double testY = 65.5;
    public static double testHeading = 180;
    public void generatePath(){
        robot.follower.setStartingPose(new Pose(8.000, 65.500, Math.toRadians(180)));

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(8.000, 65.500, Point.CARTESIAN),
                                        new Point(36.000, 65.500, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(36.000, 65.500, Point.CARTESIAN),
                                        new Point(35.000, 29.000, Point.CARTESIAN),
                                        new Point(12.000, 21.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(62.118, 22.306, Point.CARTESIAN),
                                new Point(11.859, 21.176, Point.CARTESIAN)
                        )
                )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 4
                                new BezierCurve(
                                        new Point(11.859, 21.176, Point.CARTESIAN),
                                        new Point(99.106, 17.224, Point.CARTESIAN),
                                        new Point(16.376, 13.835, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 5
                                new BezierCurve(
                                        new Point(16.376, 13.835, Point.CARTESIAN),
                                        new Point(125.082, 4.800, Point.CARTESIAN),
                                        new Point(9.318, 8.753, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build());

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierCurve(
                                        new Point(9.318, 8.753, Point.CARTESIAN),
                                        new Point(20.612, 46.871, Point.CARTESIAN),
                                        new Point(36.141, 65.224, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 7
                                new BezierCurve(
                                        new Point(36.141, 65.224, Point.CARTESIAN),
                                        new Point(14.965, 53.082, Point.CARTESIAN),
                                        new Point(10.447, 29.365, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(10.447, 29.365, Point.CARTESIAN),
                                        new Point(15.247, 63.812, Point.CARTESIAN),
                                        new Point(36.141, 66.353, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 9
                                new BezierCurve(
                                        new Point(36.141, 66.353, Point.CARTESIAN),
                                        new Point(14.965, 53.082, Point.CARTESIAN),
                                        new Point(10.447, 29.365, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 10
                                new BezierCurve(
                                        new Point(10.447, 29.365, Point.CARTESIAN),
                                        new Point(15.247, 63.812, Point.CARTESIAN),
                                        new Point(36.141, 68.800, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 11
                                new BezierCurve(
                                        new Point(36.141, 68.800, Point.CARTESIAN),
                                        new Point(14.965, 53.082, Point.CARTESIAN),
                                        new Point(10.447, 29.600, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 12
                                new BezierCurve(
                                        new Point(10.447, 29.600, Point.CARTESIAN),
                                        new Point(15.247, 63.812, Point.CARTESIAN),
                                        new Point(36.141, 71.500, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 13
                                new BezierCurve(
                                        new Point(36.141, 71.500, Point.CARTESIAN),
                                        new Point(14.965, 60.800, Point.CARTESIAN),
                                        new Point(10.447, 29.365, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build());


    }

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        timer.reset();
        TelemetryUtil.setup(telemetry);
        CommandScheduler.getInstance().reset();
        Constants.setConstants(FConstants.class, LConstants.class);
        robot = new Robot(hardwareMap, true, false);
        generatePath();

        CommandScheduler.getInstance().schedule(
                new RunCommand(() -> robot.follower.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(0)),
                        new FollowPathCommand(robot.follower, paths.get(1)),
                        new FollowPathCommand(robot.follower, paths.get(2)),
                        new FollowPathCommand(robot.follower, paths.get(3)),
                        new FollowPathCommand(robot.follower, paths.get(4)),
                        new FollowPathCommand(robot.follower, paths.get(5)),
                        new FollowPathCommand(robot.follower, paths.get(6)),
                        new FollowPathCommand(robot.follower, paths.get(7)),
                        new FollowPathCommand(robot.follower, paths.get(8)),
                        new FollowPathCommand(robot.follower, paths.get(9)),
                        new FollowPathCommand(robot.follower, paths.get(10)),
                        new FollowPathCommand(robot.follower, paths.get(11)),
                        new FollowPathCommand(robot.follower, paths.get(12)),
                        new FollowPathCommand(robot.follower, paths.get(13))

                )
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            robot.loop();

            TelemetryUtil.addData("POSE", robot.follower.getPose());
            TelemetryUtil.addData("TIMER", timer.milliseconds());

            TelemetryUtil.update();

            dashboardPoseTracker.update();
            Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
            Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
            Drawing.sendPacket();
        }
        // Cancels all commands.
        CommandScheduler.getInstance().reset();
    }
}
