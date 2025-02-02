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
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
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

        paths.add( //index 0
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(8.000, 65.500, Point.CARTESIAN),
                                        new Point(39.500, 65.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 1
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(39.500, 65.000, Point.CARTESIAN),
                                        new Point(19.000, 23.000, Point.CARTESIAN),
                                        new Point(64.000, 44.500, Point.CARTESIAN),
                                        new Point(57.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // Line 3
                                new BezierLine(
                                        new Point(57.000, 24.000, Point.CARTESIAN),
                                        new Point(20.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // Line 4
                                new BezierCurve(
                                        new Point(20.000, 24.000, Point.CARTESIAN),
                                        new Point(62.000, 35.000, Point.CARTESIAN),
                                        new Point(57.000, 13.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(57.000, 13.000, Point.CARTESIAN),
                                        new Point(20.000, 13.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // Line 6
                                new BezierCurve(
                                        new Point(20.000, 13.000, Point.CARTESIAN),
                                        new Point(61.000, 17.500, Point.CARTESIAN),
                                        new Point(57.000, 6.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(
                                // Line 7
                                new BezierCurve(
                                        new Point(57.000, 6.000, Point.CARTESIAN),
                                        new Point(-8.000, -1.000, Point.CARTESIAN),
                                        new Point(29.000, 23.000, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 2
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(22.000, 74.500, Point.CARTESIAN),
                                        new Point(39.500, 68.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 3
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 9
                                new BezierCurve(
                                        new Point(39.500, 68.000, Point.CARTESIAN),
                                        new Point(8.000, 70.000, Point.CARTESIAN),
                                        new Point(42.000, 20.000, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 4
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 10
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(21.000, 75.500, Point.CARTESIAN),
                                        new Point(39.500, 71.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add( //index 5
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 11
                                new BezierCurve(
                                        new Point(39.500, 71.000, Point.CARTESIAN),
                                        new Point(8.000, 70.000, Point.CARTESIAN),
                                        new Point(42.000, 20.000, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 6
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 12
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(20.000, 77.000, Point.CARTESIAN),
                                        new Point(39.500, 74.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 7
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 13
                                new BezierCurve(
                                        new Point(39.500, 74.000, Point.CARTESIAN),
                                        new Point(8.000, 70.000, Point.CARTESIAN),
                                        new Point(42.000, 20.000, Point.CARTESIAN),
                                        new Point(9.675, 23.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 8
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 14
                                new BezierCurve(
                                        new Point(9.675, 23.000, Point.CARTESIAN),
                                        new Point(20.000, 77.000, Point.CARTESIAN),
                                        new Point(39.500, 77.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());
        paths.add( //index 9
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 15
                                new BezierLine(
                                        new Point(39.500, 77.000, Point.CARTESIAN),
                                        new Point(28.000, 56.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(218)).build());
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
                        new FollowPathCommand(robot.follower, paths.get(0)), //going to deposit preload(high chamber)

                        new SequentialCommandGroup( //deposit preload
                                new HighChamberCommand(robot, false),
                                new WaitCommand(1000),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN), //letting go of specimen 1(preload)
                                new WaitCommand(500)
                        ),

                        new ParallelCommandGroup( //obsolete(doesn't really need to be parallel because GrabOffWallCommand will return true immediately)
                                new GrabOffWallCommand(robot), //set up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(1)) //do a lot of things(pushing all samples) and then going to pick up first specimen
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(300), //giving human player time to adjust to robot?
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED), //closing claw to pick up specimen 2
                                new WaitCommand(150),
                                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE), //lifting arm to stay clear of wall while moving
                                new WaitCommand(300)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(2)), //go to high chamber
                        new SequentialCommandGroup( //deposit specimen 2
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN), //letting go of specimen
                                new WaitCommand(200)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(3)), //going to pick up specimen 3
                                new GrabOffWallCommand(robot) //set up for next spec pickup
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(300), //giving human player time to adjust to robot
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED), //closing claw to pick up specimen
                                new WaitCommand(150),
                                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE), //lifting arm to stay clear of wall while moving
                                new WaitCommand(300)
                        ),


                        new FollowPathCommand(robot.follower, paths.get(4)), //go to high chamber + deposit
                        new SequentialCommandGroup( //deposit specimen 3
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN), //letting go of specimen
                                new WaitCommand(200)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(5)), //going to pick up specimen 4
                                new GrabOffWallCommand(robot) //set up for next spec pickup
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(300), //giving human player time to adjust to robot?
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED), //closing claw to pick up specimen 4
                                new WaitCommand(150),
                                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE), //lifting arm to stay clear of wall while moving
                                new WaitCommand(300)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(6)), //go to high chamber + deposit
                        new SequentialCommandGroup( //deposit specimen 4
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN), //letting go of specimen
                                new WaitCommand(350)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(7)), //going to pick up specimen 5
                                new GrabOffWallCommand(robot) //set up for next spec pickup
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(300), //giving human player time to adjust to robot?
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED), //closing claw to pick up specimen 5
                                new WaitCommand(150),
                                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE), //lifting arm to stay clear of wall while moving
                                new WaitCommand(300)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(8)), //go to high chamber + deposit
                        new SequentialCommandGroup( //deposit specimen 5
                                new HighChamberCommand(robot, false),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN), //letting go of specimen
                                new WaitCommand(350)
                        ),

                        //maybe we should put this stuff in a parallel command:
                        new FollowPathCommand(robot.follower, paths.get(9)), //park position/location
                        new IntakeCommand(robot) //extending intake to get the IntakeArm in the observation zone for park
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
