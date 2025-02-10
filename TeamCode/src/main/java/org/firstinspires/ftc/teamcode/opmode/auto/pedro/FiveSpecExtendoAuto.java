package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.GrabOffWallCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighChamberCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TeleOpInitializeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;
@Config
@Autonomous(group = "production")
public class FiveSpecExtendoAuto extends LinearOpMode {

    public static double testX = 27;
    public static double testY = 65.5;
    public static double testHeading = 180;
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private Robot robot;
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;

    public void generatePath() {
        robot.follower.setStartingPose(new Pose(8.000, 65.500, Math.toRadians(180)));

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // preload 1
                                new BezierLine(
                                        new Point(8.000, 65.500, Point.CARTESIAN),
                                        new Point(39.000, 75.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );


        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // sample one intake
                                new BezierCurve(
                                        new Point(39.000, 75.000, Point.CARTESIAN),
                                        new Point(16, 43, Point.CARTESIAN),
                                        new Point(28.600, 43.600, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140)).build()
        );

        //EXTEND INTAKE

        //TURN
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // turn
                                new BezierLine(
                                        new Point(28.600, 43.600, Point.CARTESIAN),
                                        new Point(28.600, 43.600, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(40)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // TURN back sample 1
                                new BezierLine(
                                        new Point(28.600, 43.600, Point.CARTESIAN),
                                        new Point(28.600, 43.600, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(140)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // go to next sample
                                new BezierLine(
                                        new Point(28.600, 43.600, Point.CARTESIAN),
                                        new Point(31.600, 36.200, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140)).build()
        );
        //INTAKE

        //TURN
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // TURN
                                new BezierLine(
                                        new Point(31.200, 36.200, Point.CARTESIAN),
                                        new Point(31.200, 36.200, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(40)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // TURN back
                                new BezierLine(
                                        new Point(31.200, 36.200, Point.CARTESIAN),
                                        new Point(31.200, 36.200, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(140)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // go to last sample
                                new BezierLine(
                                        new Point(31.200, 36.200, Point.CARTESIAN),
                                        new Point(31.600, 28.400, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140)).build()
        );

        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // turn
                                new BezierLine(
                                        new Point(31.600, 28.400, Point.CARTESIAN),
                                        new Point(31.2, 36.2, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(40)).build()
        );
        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierCurve(
                                        new Point(31.200, 36.200, Point.CARTESIAN),
                                        new Point(25.200, 33.200, Point.CARTESIAN),
                                        new Point(9.500, 34.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(180)).build()
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
                // Shouldn't need this because it's called in robot.loop()
                //new RunCommand(() -> robot.follower.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, paths.get(0)), //preload

                        new FollowPathCommand(robot.follower, paths.get(2)), //push sample 1
                        new FollowPathCommand(robot.follower, paths.get(3)), //behind sample 2

                        new FollowPathCommand(robot.follower, paths.get(4)), //push sample 2
                        new FollowPathCommand(robot.follower, paths.get(5)), //pickup sample 2



                        new FollowPathCommand(robot.follower, paths.get(6)), //go to chamber

                                new FollowPathCommand(robot.follower, paths.get(7)), //pick up sample 3



                        new TeleOpInitializeCommand(robot, false)
                )
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        waitForStart();


        robot.end();
    }


}


