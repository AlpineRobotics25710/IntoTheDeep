package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
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
import org.firstinspires.ftc.teamcode.robot.commands.HighBasketCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeEndCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;

@Config
@Autonomous(group = "production")
public class NetSideAuto extends LinearOpMode {
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private Robot robot;
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;

    public void generatePaths() {
        robot.follower.setStartingPose(new Pose(8.000, 65.500, Math.toRadians(180)));

        paths.add(
                robot.follower.pathBuilder().addPath(
                                // Line 1
                                new BezierCurve(
                                        new Point(8.000, 90.500, Point.CARTESIAN),
                                        new Point(28.480, 105.600, Point.CARTESIAN),
                                        new Point(17.120, 126.080, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .setReversed(true)
                        .build()
        );

        paths.add(
                robot.follower.pathBuilder().addPath(
                                // Line 2
                                new BezierLine(
                                        new Point(17.120, 126.080, Point.CARTESIAN),
                                        new Point(31.520, 134.080, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(210))
                        .build()
        );

        paths.add(
                robot.follower.pathBuilder().addPath(
                                // Line 3
                                new BezierLine(
                                        new Point(31.520, 134.080, Point.CARTESIAN),
                                        new Point(17.120, 126.080, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(135))
                        .build()
        );

        paths.add(
                robot.follower.pathBuilder().addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(17.120, 126.080, Point.CARTESIAN),
                                        new Point(34.240, 131.040, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                        .build()
        );

        paths.add(
                robot.follower.pathBuilder().addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(34.240, 131.040, Point.CARTESIAN),
                                        new Point(17.120, 126.080, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build()
        );

        paths.add(
                robot.follower.pathBuilder().addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(17.120, 126.080, Point.CARTESIAN),
                                        new Point(34.240, 121.440, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                        .build()
        );

        paths.add(
                robot.follower.pathBuilder().addPath(
                                // Line 7
                                new BezierLine(
                                        new Point(34.240, 121.440, Point.CARTESIAN),
                                        new Point(17.120, 126.080, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build()
        );

        paths.add(
                robot.follower.pathBuilder().addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(17.120, 126.080, Point.CARTESIAN),
                                        new Point(55.520, 127.040, Point.CARTESIAN),
                                        new Point(62.880, 97.440, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                        .build()
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
        generatePaths();

        CommandGroupBase intakeAndTransfer = new SequentialCommandGroup(
                new IntakeCommand(robot),
                new TransferCommand(robot),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED),
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF)
        );

        CommandGroupBase deposit = new SequentialCommandGroup(
                new HighBasketCommand(robot, false),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                new OuttakeRetractCommand(robot)
        );

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(0)), // preload
                        deposit,    // deposit preload
                        new FollowPathCommand(robot.follower, paths.get(1)), // sample 1
                        intakeAndTransfer,
                        new FollowPathCommand(robot.follower, paths.get(2)), // deposit sample 1
                        deposit,
                        new FollowPathCommand(robot.follower, paths.get(3)), // sample 2
                        intakeAndTransfer,
                        new FollowPathCommand(robot.follower, paths.get(4)), // deposit sample 2
                        deposit,
                        new FollowPathCommand(robot.follower, paths.get(5)), // sample 3
                        intakeAndTransfer,
                        new FollowPathCommand(robot.follower, paths.get(6)), // deposit sample 3
                        deposit,
                        new FollowPathCommand(robot.follower, paths.get(7)) // park
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
