package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;

@Config
@Autonomous(group = "production")
public class NetsideAuto extends LinearOpMode {
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private Robot robot;
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;

    public void generatePaths() {
        robot.follower.setStartingPose(new Pose(8, 113.500, Math.toRadians(-90)));

        paths.add( //index 0
                robot.follower.pathBuilder()
                        .addPath( //line 1
                                new BezierLine(
                                        new Point(8.000, 113.500, Point.CARTESIAN),
                                        new Point(15.5, 128.5, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                        .build()
        );

        paths.add( //index 1
                robot.follower.pathBuilder()
                        .addPath( //line 2
                                new BezierLine(
                                        new Point(15.5, 128.5, Point.CARTESIAN),
                                        new Point(35.000, 87.500, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(73))
                        .build()
        );

        paths.add( //index 2
                robot.follower.pathBuilder()
                        .addPath( //line 3
                                new BezierLine(
                                        new Point(35.000, 87.500, Point.CARTESIAN),
                                        new Point(39.000, 97.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(73))
                        .build()
        );

        paths.add( //index 3
                robot.follower.pathBuilder()
                        .addPath( //line 4
                                new BezierLine(
                                        new Point(39.000, 97.500, Point.CARTESIAN),
                                        new Point(15.5, 128.5, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(73), Math.toRadians(-45))
                        .build()
        );

        paths.add( //index 4
                robot.follower.pathBuilder()
                        .addPath( //line 5
                                new BezierLine(
                                        new Point(15.5, 128.5, Point.CARTESIAN),
                                        new Point(35.000, 93.500, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(73))
                        .build()
        );

        paths.add( //index 5
                robot.follower.pathBuilder()
                        .addPath( //line 6
                                new BezierLine(
                                        new Point(35.000, 93.500, Point.CARTESIAN),
                                        new Point(39.000, 103.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(73))
                        .build()
        );

        paths.add( //index 6
                robot.follower.pathBuilder()
                        .addPath( //line 7
                                new BezierLine(
                                        new Point(39.000, 103.500, Point.CARTESIAN),
                                        new Point(15.5, 128.5, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(73), Math.toRadians(-45))
                        .build()
        );

        paths.add( //index 7
                robot.follower.pathBuilder()
                        .addPath( //line 8
                                new BezierLine(
                                        new Point(15.5, 128.5, Point.CARTESIAN),
                                        new Point(35.000, 99.500, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(73))
                        .build()
        );

        paths.add( //index 8
                robot.follower.pathBuilder()
                        .addPath( //line 9
                                new BezierLine(
                                        new Point(35.000, 99.500, Point.CARTESIAN),
                                        new Point(39.000, 109.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(73))
                        .build()
        );

        paths.add( //index 9
                robot.follower.pathBuilder()
                        .addPath( //line 10
                                new BezierLine(
                                        new Point(39.000, 109.500, Point.CARTESIAN),
                                        new Point(15.5, 128.5, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(73), Math.toRadians(-45))
                        .build()
        );

        paths.add( //index 10
                robot.follower.pathBuilder()
                        .addPath( //line 11
                                new BezierCurve(
                                        new Point(15.5, 128.5, Point.CARTESIAN),
                                        new Point(60.000, 120.000, Point.CARTESIAN),
                                        new Point(60.000, 95.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
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
        robot = new Robot(hardwareMap, true);
        generatePaths();

        CommandGroupBase intakeAndTransfer = new SequentialCommandGroup( //redundant apparantly so just use TransferCommand()
                new IntakeCommand(robot, IntakeArm.IntakeArmState.INTAKE),
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD),
                new WaitCommand(500),
                new TransferCommand(robot),
                new WaitCommand(500)
        );

        CommandGroupBase deposit = new SequentialCommandGroup(
                new HighBasketCommand(robot),
                new WaitCommand(800),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                new WaitCommand(300),
                new OuttakeRetractCommand(robot)
        );

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(0)), // deposit preload

                        deposit,

                        new FollowPathCommand(robot.follower, paths.get(1)), // going to pos to intake sample 1
                        new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD),

                        new FollowPathCommand(robot.follower, paths.get(2)), // moving forward to intake sample 1

                        new TransferCommand(robot),

                        new FollowPathCommand(robot.follower, paths.get(3)), // going to deposit sample 1

                        deposit,

                        new FollowPathCommand(robot.follower, paths.get(4)), // going to pos to intake sample 2
                        new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD),

                        new FollowPathCommand(robot.follower, paths.get(5)), // moving forward to intake sample 2

                        new TransferCommand(robot),

                        new FollowPathCommand(robot.follower, paths.get(6)), // going to deposit sample 2

                        deposit,

                        new FollowPathCommand(robot.follower, paths.get(7)), // going to pos to intake sample 3
                        new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD),

                        new FollowPathCommand(robot.follower, paths.get(8)), // moving forward to intake sample 3

                        new TransferCommand(robot),

                        new FollowPathCommand(robot.follower, paths.get(9)), // going to deposit sample 3

                        deposit,

                        new FollowPathCommand(robot.follower, paths.get(10)) // park
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
