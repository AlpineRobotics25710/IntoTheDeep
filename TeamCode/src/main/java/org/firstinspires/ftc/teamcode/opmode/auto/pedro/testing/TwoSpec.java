package org.firstinspires.ftc.teamcode.opmode.auto.pedro.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandGroupBase;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.GrabOffWallCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighChamberCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.OuttakeIntermediateCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;

@Config
@Autonomous(group="testing")
public class TwoSpec extends LinearOpMode {
    private ElapsedTime timer;
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private DashboardPoseTracker dashboardPoseTracker;
    Robot robot;
    public static double testScore = 40;
    public static double testGrab = 35;
    public static double testGrabDistance = 11;

    public static final long CLAW_DEPOSIT_DELAY = 100;
    public static final long DEPOSIT_DELAY = 200;

    public void generatePath(){
        robot.follower.setStartingPose(new Pose(8.000, 65.500, Math.toRadians(180)));

        paths.add( //index 0
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(8.000, 65.500, Point.CARTESIAN),
                                        new Point(testScore, 75.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 1
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(testScore, 75.00, Point.CARTESIAN),
                                        new Point(11.000, 66.000, Point.CARTESIAN),
                                        new Point(30.000, testGrab + 1.5, Point.CARTESIAN),
                                        new Point(testGrabDistance, testGrab, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 2
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 3
                                new BezierCurve(
                                        new Point(testGrabDistance, testGrab, Point.CARTESIAN),
                                        new Point(22.000, 74.500, Point.CARTESIAN),
                                        new Point(testScore, 74.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .setZeroPowerAccelerationMultiplier(4)
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
        generatePath();

        CommandGroupBase deposit = new SequentialCommandGroup(
                //depositing specimen at high chamber
                new HighChamberCommand(robot),
                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open

        );

        CommandGroupBase pickUp = new SequentialCommandGroup( //grabbing specimen and preparing to deposit
                new WaitCommand(150), //WE CAN REMOVE THIS LATER
                new OuttakeIntermediateCommand(robot)
                //new WaitCommand(350) //dont need this i think?
        );

        CommandScheduler.getInstance().schedule(
                new RunCommand(() -> robot.follower.update()), //shouldn't need this cus its called in robot.loop()??? idk ill keep it here fo rnow
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeIntermediateCommand(robot, true),
                                new FollowPathCommand(robot.follower, paths.get(0))//go to deposit preload
                        ),

                        deposit,

                        new ParallelCommandGroup(
                            new FollowPathCommand(robot.follower, paths.get(1)),
                            new GrabOffWallCommand(robot)
                        ),

                        pickUp,

                        new FollowPathCommand(robot.follower, paths.get(2)),

                        deposit
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
