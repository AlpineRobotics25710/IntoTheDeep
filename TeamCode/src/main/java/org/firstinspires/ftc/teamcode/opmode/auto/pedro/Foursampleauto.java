package org.firstinspires.ftc.teamcode.opmode.auto.pedro;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.commands.GrabOffWallCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighChamberCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.robot.commands.OuttakeIntermediateCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;

@Config
@Autonomous(group="production")
public class Foursampleauto {
    private ElapsedTime timer;
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private DashboardPoseTracker dashboardPoseTracker;
    Robot robot;
    public static double testScore = 41.500;
    public static double testGrab = 35.000;
    public static double testGrabDistance = 10.25;
    public static double specScore = 78.500;

    public static final long CLAW_DEPOSIT_DELAY = 100;
    public static final long DEPOSIT_DELAY = 200;
    public void generatePath(){
        robot.follower.setStartingPose(new Pose(8.000, 65.500, Math.toRadians(180)));

        paths.add( //index 0
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(8.000, 100.000, Point.CARTESIAN),
                                        new Point(63.000, 97.700, Point.CARTESIAN),
                                        new Point(22.300, 122.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build()
        );

        paths.add( //index 1
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(22.300, 122.000, Point.CARTESIAN),
                                        new Point(34.600, 91.000, Point.CARTESIAN),
                                        new Point(53.500, 95.700, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                        .build()
        );

        paths.add( //index 2
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(53.500, 95.700, Point.CARTESIAN),
                                        new Point(41.400, 86.100, Point.CARTESIAN),
                                        new Point(22.300, 121.900, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                        .build()
        );

        paths.add( //index 3
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(22.300, 121.900, Point.CARTESIAN),
                                        new Point(52.000, 74.000, Point.CARTESIAN),
                                        new Point(54.800, 107.900, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                        .build()
        );

        paths.add( //index 4
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(54.800, 107.900, Point.CARTESIAN),
                                        new Point(37.700, 99.700, Point.CARTESIAN),
                                        new Point(22.100, 121.700, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                        .build()
        );

        paths.add( //index 5
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(22.100, 121.700, Point.CARTESIAN),
                                        new Point(30.700, 88.300, Point.CARTESIAN),
                                        new Point(54.000, 117.700, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                        .build()
        );

        paths.add( //index 6
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(54.000, 117.700, Point.CARTESIAN),
                                        new Point(33.400, 100.200, Point.CARTESIAN),
                                        new Point(22.300, 121.700, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                        .build()
        );

        paths.add( //index 7
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(22.300, 121.700, Point.CARTESIAN),
                                        new Point(79.000, 111.400, Point.CARTESIAN),
                                        new Point(75.500, 85.300, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
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

        CommandGroupBase intake = new SequentialCommandGroup(
                //depositing specimen at high chamber
                new IntakeCommand(robot),
                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                new IntakeRetractCommand(robot, IntakeArm.IntakeArmState. ),
                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open

        );

    }
}
