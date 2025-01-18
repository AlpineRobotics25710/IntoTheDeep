package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.GrabOffWallCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighChamberCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;

@Config
@Autonomous(group="production")
public class FourSpecAuto extends LinearOpMode {
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
                                        new Point(39.500, 68.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );


        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(39.500, 68.000, Point.CARTESIAN),
                                        new Point(3.000, 33.000, Point.CARTESIAN),
                                        new Point(67.000, 43.000, Point.CARTESIAN),
                                        new Point(60.000, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 3
                                new BezierCurve(
                                        new Point(60.000, 23.000, Point.CARTESIAN),
                                        new Point(9.000, 18.000, Point.CARTESIAN),
                                        new Point(15, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 4
                                new BezierCurve(
                                        new Point(15, 23.000, Point.CARTESIAN),
                                        new Point(76.000, 35.000, Point.CARTESIAN),
                                        new Point(60.000, 12.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(60.000, 12.000, Point.CARTESIAN),
                                        new Point(9.75, 12.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierCurve(
                                        new Point(9.75, 12.000, Point.CARTESIAN),
                                        new Point(20.000, 60.000, Point.CARTESIAN),
                                        new Point(39.500, 64.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 7
                                new BezierCurve(
                                        new Point(39.500, 64.000, Point.CARTESIAN),
                                        new Point(4.000, 67.000, Point.CARTESIAN),
                                        new Point(46.000, 20.000, Point.CARTESIAN),
                                        new Point(9.750, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(9.75, 23.000, Point.CARTESIAN),
                                        new Point(20.000, 60.000, Point.CARTESIAN),
                                        new Point(39.500, 68.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 9
                                new BezierCurve(
                                        new Point(39.500, 68.000, Point.CARTESIAN),
                                        new Point(2.500, 71.000, Point.CARTESIAN),
                                        new Point(50.000, 21.500, Point.CARTESIAN),
                                        new Point(9.700, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 10
                                new BezierCurve(
                                        new Point(9.75, 23.000, Point.CARTESIAN),
                                        new Point(20.000, 60.000, Point.CARTESIAN),
                                        new Point(39.500, 72.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 11
                                new BezierCurve(
                                        new Point(39.500, 72.000, Point.CARTESIAN),
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
                new RunCommand(() -> robot.follower.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(0)), //preload

                        new SequentialCommandGroup( //deposit preload
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(250)
                        ),

                        new ParallelCommandGroup( //obsolete
                            new GrabOffWallCommand(robot), //set up for next spec pickup
                            new FollowPathCommand(robot.follower, paths.get(1)) //behind sample 1 and outtake in front
                        ),

                        new FollowPathCommand(robot.follower, paths.get(2)), //push sample 1
                        new FollowPathCommand(robot.follower, paths.get(3)), //behind sample 2

                        new FollowPathCommand(robot.follower, paths.get(4)), //push sample 2 and pickup
                        new SequentialCommandGroup(
                            new WaitCommand(100), //WE CAN REMOVE THIS LATER
                            new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED), //closing claw to pick up specimen
                            new WaitCommand(150),
                            new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE), //lifting arm to stay clear of wall while moving
                            new WaitCommand(300)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(5)), //go to chamber
                        new SequentialCommandGroup( //deposit sample 2
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(250)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(6)), //pick up sample 3
                                new GrabOffWallCommand(robot) //set up for next spec pickup
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(100), //WE CAN REMOVE THIS LATER
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED), //closing claw to pick up specimen
                                new WaitCommand(150),
                                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE), //lifting arm to stay clear of wall while moving
                                new WaitCommand(300)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(7)), //go to chamber + deposit
                        new SequentialCommandGroup( //deposit sample 3
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(250)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(8)), //pick up sample 3
                                new GrabOffWallCommand(robot) //set up for next spec pickup
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(100), //WE CAN REMOVE THIS LATER
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED), //closing claw to pick up specimen
                                new WaitCommand(150),
                                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE), //lifting arm to stay clear of wall while moving
                                new WaitCommand(300)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(9)), //go to chamber + deposit
                        new SequentialCommandGroup( //deposit sample 4
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(350)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(10)), //pick up sample 3
                                new GrabOffWallCommand(robot) //set up for next spec pickup
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
        // Cancels all commands.
        CommandScheduler.getInstance().reset();
    }
}
