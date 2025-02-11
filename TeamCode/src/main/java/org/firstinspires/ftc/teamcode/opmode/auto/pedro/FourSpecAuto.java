package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import org.firstinspires.ftc.teamcode.robot.commands.GrabOffWallCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighChamberCommand;
import org.firstinspires.ftc.teamcode.robot.commands.OuttakeIntermediateCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TeleOpInitializeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;

@Config
@Autonomous(group = "production")
public class FourSpecAuto extends LinearOpMode {
    public static double testX = 27;
    public static double testY = 65.5;
    public static double testHeading = 180;
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private Robot robot;
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;

    public void generatePath() {
        robot.follower.setStartingPose(new Pose(8.000, 65.500, Math.toRadians(180)));

        paths.add( //index 0
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(8.000, 65.500, Point.CARTESIAN),
                                        new Point(37.500, 68.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );


        paths.add( //index 1
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(37.500, 68.000, Point.CARTESIAN),
                                        new Point(3.000, 33.000, Point.CARTESIAN),
                                        new Point(67.000, 43.000, Point.CARTESIAN),
                                        new Point(60.000, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // Line 3
                                new BezierCurve(
                                        new Point(60.000, 23.000, Point.CARTESIAN),
                                        new Point(20.157, 18.060, Point.CARTESIAN),
                                        new Point(20.000, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // Line 4
                                new BezierCurve(
                                        new Point(15, 23.000, Point.CARTESIAN),
                                        new Point(73, 43, Point.CARTESIAN),
                                        new Point(60.000, 16, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                //line 5
                                new BezierCurve(
                                        new Point(60.000, 16.000, Point.CARTESIAN),
                                        new Point(54.826, 18.060, Point.CARTESIAN),
                                        new Point(18.157, 10.965, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // something i'm liteally high on smth
                                new BezierCurve(
                                        new Point(18.157, 10.965, Point.CARTESIAN),
                                        new Point(25.962, 29.510, Point.CARTESIAN),
                                        new Point(9.675, 26.607, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 2
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierCurve(
                                        new Point(9.675, 12.000, Point.CARTESIAN),
                                        new Point(20.000, 60.000, Point.CARTESIAN),
                                        new Point(37.500, 71, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 3
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 7
                                new BezierCurve(
                                        new Point(37.500, 71, Point.CARTESIAN),
                                        new Point(4.000, 67.000, Point.CARTESIAN),
                                        new Point(46.000, 20.000, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 4
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(20.000, 60.000, Point.CARTESIAN),
                                        new Point(37.500, 74, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 5
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 9
                                new BezierCurve(
                                        new Point(37.500, 74, Point.CARTESIAN),
                                        new Point(2.500, 71.000, Point.CARTESIAN),
                                        new Point(50.000, 21.500, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 6
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 10
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(20.000, 60.000, Point.CARTESIAN),
                                        new Point(37.500, 77, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 7
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 11
                                new BezierCurve(
                                        new Point(37.500, 77, Point.CARTESIAN),
                                        new Point(12.000, 73.000, Point.CARTESIAN),
                                        new Point(10.000, 10.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
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
                // Shouldn't need this because it's called in robot.loop()
                //new RunCommand(() -> robot.follower.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(0)), //preload

                        new SequentialCommandGroup( //deposit preload
                                new HighChamberCommand(robot, false),
                                new WaitCommand(1000),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(500)
                        ),

                        new ParallelCommandGroup( //obsolete(doesn't really need to be parallel because GrabOffWallCommand will return true immediately)
                                new GrabOffWallCommand(robot), //set up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(1)) //do a lot of things(pushing all samples) and then going to pick up first specimen
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(150), //WE CAN REMOVE THIS LATER
                                new OuttakeIntermediateCommand(robot),
                                new WaitCommand(300)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(2)), //go to chamber
                        new SequentialCommandGroup( //deposit specimen 1
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(250)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(3)), //going to pick up specimen 2
                                new GrabOffWallCommand(robot) //set up for next spec pickup
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(150), //WE CAN REMOVE THIS LATER
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED), //closing claw to pick up specimen 2
                                new WaitCommand(225),
                                new OuttakeIntermediateCommand(robot),
                                new WaitCommand(300)
                        ),


                        new FollowPathCommand(robot.follower, paths.get(4)), //go to chamber + deposit
                        new SequentialCommandGroup( //deposit specimen 3
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(250)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(5)), //going to pick up specimen 3
                                new GrabOffWallCommand(robot) //set up for next spec pickup
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(150), //WE CAN REMOVE THIS LATER
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED), //closing claw to pick up specimen 4
                                new WaitCommand(225),
                                new OuttakeIntermediateCommand(robot),
                                new WaitCommand(300)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(6)), //go to chamber + deposit
                        new SequentialCommandGroup( //deposit specimen 4
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(350)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(7)), //park
                                new GrabOffWallCommand(robot) //set up for tele op ig?
                        ),

                        new TeleOpInitializeCommand(robot, false)
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
