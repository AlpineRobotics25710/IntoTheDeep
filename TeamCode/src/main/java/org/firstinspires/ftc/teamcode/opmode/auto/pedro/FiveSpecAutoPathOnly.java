package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;

@Config
@Autonomous(group="production")
public class FiveSpecAutoPathOnly extends LinearOpMode {
    private ElapsedTime timer;
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private DashboardPoseTracker dashboardPoseTracker;
    Robot robot;
    public static double testX = 27;
    public static double testY = 65.5;
    public static double testHeading = 180;
    public void generatePath(){
        robot.follower.setStartingPose(new Pose(8.000, 65.500, Math.toRadians(180)));

        paths.add( //index 0
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(8.000, 65.500, Point.CARTESIAN),
                                        new Point(39.000, 77.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 1
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(39.000, 77.000, Point.CARTESIAN),
                                        new Point(12.500, 16.500, Point.CARTESIAN),
                                        new Point(64.000, 44.500, Point.CARTESIAN),
                                        new Point(57.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 2
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 3
                                new BezierLine(
                                        new Point(57.000, 24.000, Point.CARTESIAN),
                                        new Point(20.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).setZeroPowerAccelerationMultiplier(10).build());
        paths.add( //index 3
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 4
                                new BezierCurve(
                                        new Point(20.000, 24.000, Point.CARTESIAN),
                                        new Point(62.500, 35.000, Point.CARTESIAN),
                                        new Point(57.000, 13.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 4
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(57.000, 13.000, Point.CARTESIAN),
                                        new Point(20.000, 13.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).setZeroPowerAccelerationMultiplier(10).build());
        paths.add( //index 5
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierCurve(
                                        new Point(20.000, 13.000, Point.CARTESIAN),
                                        new Point(61.000, 17.500, Point.CARTESIAN),
                                        new Point(57.000, 9.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 6
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 7
                                new BezierCurve(
                                        new Point(57.000, 9.000, Point.CARTESIAN),
                                        new Point(45.000, 10.000, Point.CARTESIAN),
                                        new Point(-3.000, 3.000, Point.CARTESIAN),
                                        new Point(20.500, 19.500, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 7
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(22.000, 74.500, Point.CARTESIAN),
                                        new Point(38.500, 74.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 8
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 9
                                new BezierCurve(
                                        new Point(38.500, 74.000, Point.CARTESIAN),
                                        new Point(11.000, 66.000, Point.CARTESIAN),
                                        new Point(30.000, 25.500, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 9
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 10
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(17.500, 70.500, Point.CARTESIAN),
                                        new Point(38.500, 71.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add( //index 10
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 11
                                new BezierCurve(
                                        new Point(38.500, 71.000, Point.CARTESIAN),
                                        new Point(10.000, 65.000, Point.CARTESIAN),
                                        new Point(30.000, 25.500, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 11
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 12
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(17.500, 75.000, Point.CARTESIAN),
                                        new Point(38.500, 68.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 12
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 13
                                new BezierCurve(
                                        new Point(38.500, 68.000, Point.CARTESIAN),
                                        new Point(11.500, 65.600, Point.CARTESIAN),
                                        new Point(30.000, 25.000, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 13
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 14
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(17.250, 64.250, Point.CARTESIAN),
                                        new Point(38.500, 65.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 14
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 15
                                new BezierLine(
                                        new Point(38.500, 65.000, Point.CARTESIAN),
                                        new Point(20.000, 54.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230)).build());
    }

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        timer.reset();
        TelemetryUtil.setup(telemetry);
        CommandScheduler.getInstance().reset();
        Constants.setConstants(FConstants.class, LConstants.class);
        robot = new Robot(hardwareMap, true);
        generatePath();
        CommandScheduler.getInstance().schedule(
                new RunCommand(() -> robot.follower.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(0)), //got to deposit preload

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

                        new FollowPathCommand(robot.follower, paths.get(13)),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(14)), //park position/location
                                new IntakeCommand(robot) //extending intake to get the IntakeArm in the observation zone for park
                        )
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
        robot.end();
    }
}
