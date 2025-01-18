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
public class TwoSpecAuto extends LinearOpMode {
    private ElapsedTime timer;
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private DashboardPoseTracker dashboardPoseTracker;
    Robot robot;
    public static double testX = 27;
    public static double testY = 65.5;
    public static double testHeading = 180;
    public void generatePath() {
        robot.follower.setStartingPose(new Pose(8.000, 65.500, Math.toRadians(180)));

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(8.000, 60.000, Point.CARTESIAN),
                                        new Point(38.000, 60.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );


        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(38.000, 60.000, Point.CARTESIAN),
                                        new Point(29.000, 26.000, Point.CARTESIAN),
                                        new Point(12.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 3
                                new BezierCurve(
                                        new Point(12.000, 24.000, Point.CARTESIAN),
                                        new Point(20.000, 60.000, Point.CARTESIAN),
                                        new Point(38.000, 64.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 4
                                new BezierCurve(
                                        new Point(38.000, 64.000, Point.CARTESIAN),
                                        new Point(30.000, 28.000, Point.CARTESIAN),
                                        new Point(9.500, 10.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

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
                        new FollowPathCommand(robot.follower, paths.get(4))
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
