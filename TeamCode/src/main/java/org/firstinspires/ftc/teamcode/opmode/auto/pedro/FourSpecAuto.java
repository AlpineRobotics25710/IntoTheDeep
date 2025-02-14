package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

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
@Autonomous(group="production")
public class FourSpecAuto extends LinearOpMode {
    private ElapsedTime timer;
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private DashboardPoseTracker dashboardPoseTracker;
    Robot robot;
    public static double testScore = 41;
    public static double testGrab = 35;
    public static double testGrabDistance = 10.5;

    public static final long CLAW_DEPOSIT_DELAY = 100;
    public static final long DEPOSIT_DELAY = 200;
    public void generatePath(){
        robot.follower.setStartingPose(new Pose(8.000, 65.500, Math.toRadians(180)));

        paths.add( //index 0
                robot.follower.pathBuilder()
                        .addPath( //line 1
                                new BezierLine(
                                        new Point(8.000, 65.500, Point.CARTESIAN),
                                        new Point(testScore, 77.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .setZeroPowerAccelerationMultiplier(3)
                        .build()
        );

        paths.add( //index 1
                robot.follower.pathBuilder()
                        .addPath( //line 2
                                new BezierCurve(
                                        new Point(testScore, 77.000, Point.CARTESIAN),
                                        new Point(12.500, 16.500, Point.CARTESIAN),
                                        new Point(64.000, 44.500, Point.CARTESIAN),
                                        new Point(57.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 2
                robot.follower.pathBuilder()
                        .addPath( //line 3
                                new BezierLine(
                                        new Point(57.000, 24.000, Point.CARTESIAN),
                                        new Point(20.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 3
                robot.follower.pathBuilder()
                        .addPath( //line 4
                                new BezierCurve(
                                        new Point(20.000, 24.000, Point.CARTESIAN),
                                        new Point(62.750, 27.000, Point.CARTESIAN),
                                        new Point(80.000, 7.500, Point.CARTESIAN),
                                        new Point(20.000, 13.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 4
                robot.follower.pathBuilder()
                        .addPath( //line 5
                                new BezierCurve(
                                        new Point(20.000, 13.000, Point.CARTESIAN),
                                        new Point(11.500, 15.000, Point.CARTESIAN),
                                        new Point(17.000, 35.500, Point.CARTESIAN),
                                        new Point(10.500, testGrab, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 5
                robot.follower.pathBuilder()
                        .addPath( //line 6
                                new BezierCurve(
                                        new Point(10.500, testGrab, Point.CARTESIAN),
                                        new Point(18.500, 66.000, Point.CARTESIAN),
                                        new Point(testScore, 74.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .setZeroPowerAccelerationMultiplier(3)
                        .build()
        );

        paths.add( //index 6
                robot.follower.pathBuilder()
                        .addPath( //line 7
                                new BezierCurve(
                                        new Point(testScore, 74.500, Point.CARTESIAN),
                                        new Point(testGrabDistance, 66.000, Point.CARTESIAN),
                                        new Point(27.500, 35.500, Point.CARTESIAN),
                                        new Point(10.500, testGrab, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 7
                robot.follower.pathBuilder()
                        .addPath( //line 8
                                new BezierCurve(
                                        new Point(10.500, testGrab, Point.CARTESIAN),
                                        new Point(17.500, 64.000, Point.CARTESIAN),
                                        new Point(testScore, 72.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .setZeroPowerAccelerationMultiplier(3)
                        .build()
        );

        paths.add( //index 8
                robot.follower.pathBuilder()
                        .addPath( //line 9
                                new BezierCurve(
                                        new Point(testScore, 72.000, Point.CARTESIAN),
                                        new Point(10.000, 65.000, Point.CARTESIAN),
                                        new Point(27.250, 35.750, Point.CARTESIAN),
                                        new Point(10.500, testGrab, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 9
                robot.follower.pathBuilder()
                        .addPath( //line 10
                                new BezierCurve(
                                        new Point(10.500, testGrab, Point.CARTESIAN),
                                        new Point(21.500, 62.000, Point.CARTESIAN),
                                        new Point(testScore, 69.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .setZeroPowerAccelerationMultiplier(3)
                        .build()
        );

        paths.add( //index 10
                robot.follower.pathBuilder()
                        .addPath( //line 11
                                new BezierCurve(
                                        new Point(testScore, 69.500, Point.CARTESIAN),
                                        new Point(11.500, 65.600, Point.CARTESIAN),
                                        new Point(27.000, 36.000, Point.CARTESIAN),
                                        new Point(10.500, testGrab, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 11
                robot.follower.pathBuilder()
                        .addPath( //line 12
                                new BezierCurve(
                                        new Point(10.500, testGrab, Point.CARTESIAN),
                                        new Point(19.500, 60.000, Point.CARTESIAN),
                                        new Point(testScore, 67.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .setZeroPowerAccelerationMultiplier(3)
                        .build()
        );

        paths.add( //index 12
                robot.follower.pathBuilder()
                        .addPath( //line 13
                                new BezierLine(
                                        new Point(testScore, 67.000, Point.CARTESIAN),
                                        new Point(20.000, 54.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
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

                        new ParallelCommandGroup( //preping for next spec pick up and going to push sample 1
                                new GrabOffWallCommand(robot), //set up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(1)) //going to position to push sample 1
                        ),

                        new FollowPathCommand(robot.follower, paths.get(2)), //pushing sample 1

                        new FollowPathCommand(robot.follower, paths.get(3)), //going to position to push sample 2

                        new FollowPathCommand(robot.follower, paths.get(4)), //pushing sample 2

                        new FollowPathCommand(robot.follower, paths.get(6)), //pushing sample 3 and going to pick up specimen 2

                        pickUp,

                        new FollowPathCommand(robot.follower, paths.get(7)), //going to high chamber to deposit specimen 2

                        deposit,

                        new ParallelCommandGroup( //going back to pick up specimen 3
                                new GrabOffWallCommand(robot), //set outtake up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(8)) //do a lot of things(pushing all samples) and then going to pick up first specimen
                        ),

                        pickUp,

                        new FollowPathCommand(robot.follower, paths.get(9)), //going to high chamber to deposit specimen 3

                        deposit,

                        new ParallelCommandGroup( //going back to pick up specimen 4
                                new GrabOffWallCommand(robot), //set outtake up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(10)) //do a lot of things(pushing all samples) and then going to pick up first specimen
                        ),

                        pickUp,

                        new FollowPathCommand(robot.follower, paths.get(11)), //going to high chamber to deposit specimen 4

                        deposit,

                        new ParallelCommandGroup( //going back to pick up specimen 5
                                new GrabOffWallCommand(robot), //set outtake up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(10)) //do a lot of things(pushing all samples) and then going to pick up first specimen
                        ),

                        pickUp,

                        new FollowPathCommand(robot.follower, paths.get(13)), //going to high chamber to deposit specimen 5

                        deposit,

                        new ParallelCommandGroup(
                                new IntakeCommand(robot, IntakeArm.IntakeArmState.INTAKE), //extending intake to get the IntakeArm in the observation zone for park
                                new FollowPathCommand(robot.follower, paths.get(14)) //park position/location
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
