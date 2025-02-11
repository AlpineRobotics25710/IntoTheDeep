package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.robot.commands.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.robot.commands.OuttakeIntermediateCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeEndCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

import java.util.ArrayList;
@Config
@Autonomous(group = "production")
public class FiveSpecExtendoPush extends LinearOpMode {

    public static double testScore = 39.25;
    public static double testGrab = 35;
    public static double testGrabDistance = 10;

    public static double facingSampleAngle = 320;
    public static double depositSampleAngle = 220;

    public static final long CLAW_DEPOSIT_DELAY = 100;
    public static final long DEPOSIT_DELAY = 150;
    private final ArrayList<PathChain> paths = new ArrayList<PathChain>();
    private Robot robot;
    private ElapsedTime timer;
    private DashboardPoseTracker dashboardPoseTracker;

    public void generatePath() {
        robot.follower.setStartingPose(new Pose(8.250, 65.500, Math.toRadians(180)));

        paths.add( //index 0
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(8.500, 65.500, Point.CARTESIAN),
                                        new Point(38.500, 77.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );


        paths.add( //index 1
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(38.500, 77.000, Point.CARTESIAN),
                                        new Point(30.000, 50.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(300))
                        .build()
        );

        //EXTEND INTAKE

        //TURN
        paths.add( //index 2
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(30.000, 50.000, Point.CARTESIAN),
                                        new Point(30.000, 40.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(300))
                        .build()
        );

        paths.add( //index 3
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(30.000, 50.000, Point.CARTESIAN),
                                        new Point(30.000, 40.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(300))
                        .build()
        );

        paths.add( //index 4
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(30.000, 40.000, Point.CARTESIAN),
                                        new Point(30.000, 40.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(235))
                        .build()
        );
        //INTAKE

        //TURN
        paths.add( //index 5
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(30.000, 40.000, Point.CARTESIAN),
                                        new Point(54.000, 36.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(270))
                        .build()
        );

        paths.add( //index 6
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(54.000, 36.000, Point.CARTESIAN),
                                        new Point(20.000, 36.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                        .build()
        );

        paths.add( //index 7
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(20.000, 36.000, Point.CARTESIAN),
                                        new Point(9.000, 36.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 8
                robot.follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Point(9.000, 36.000, Point.CARTESIAN),
                                        new Point(38.500, 74.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );
        paths.add( //index 9
                robot.follower.pathBuilder()
                        .addPath( //line 10
                                new BezierCurve(
                                        new Point(38.500, 74.500, Point.CARTESIAN),
                                        new Point(20.000, 69.000, Point.CARTESIAN),
                                        new Point(30.000, 35.000, Point.CARTESIAN),
                                        new Point(9.000, 36.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );
        paths.add( //index 10
                robot.follower.pathBuilder()
                        .addPath( //line 11
                                new BezierLine(
                                        new Point(9.000, 36.000, Point.CARTESIAN),
                                        new Point(38.500, 72.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 11
                robot.follower.pathBuilder()
                        .addPath( //line 12
                                new BezierCurve(
                                        new Point(38.500, 72.000, Point.CARTESIAN),
                                        new Point(20.000, 66.500, Point.CARTESIAN),
                                        new Point(31.000, 36.000, Point.CARTESIAN),
                                        new Point(9.000, 36.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 12
                robot.follower.pathBuilder()
                        .addPath( //line 13
                                new BezierLine(
                                        new Point(9.000, 36.000, Point.CARTESIAN),
                                        new Point(38.500, 69.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 13
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 10
                                new BezierCurve(
                                        new Point(testGrabDistance, testGrab, Point.CARTESIAN),
                                        new Point(17.500, 73.000, Point.CARTESIAN),
                                        new Point(testScore, 72.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 14
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 11
                                new BezierCurve(
                                        new Point(testScore, 72.000, Point.CARTESIAN),
                                        new Point(10.000, 65.000, Point.CARTESIAN),
                                        new Point(30.000, testGrab + 1, Point.CARTESIAN),
                                        new Point(testGrabDistance, testGrab, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 15
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 12
                                new BezierCurve(
                                        new Point(testGrabDistance, testGrab, Point.CARTESIAN),
                                        new Point(17.500, 70.500, Point.CARTESIAN),
                                        new Point(testScore, 69.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 16
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 13
                                new BezierCurve(
                                        new Point(testScore, 69.500, Point.CARTESIAN),
                                        new Point(11.500, 65.600, Point.CARTESIAN),
                                        new Point(30.000, testGrab + 1, Point.CARTESIAN),
                                        new Point(testGrabDistance, testGrab, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 17
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 14
                                new BezierCurve(
                                        new Point(testGrabDistance, testGrab, Point.CARTESIAN),
                                        new Point(17.250, 64.250, Point.CARTESIAN),
                                        new Point(testScore, 67.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );

        paths.add( //index 18
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 15
                                new BezierLine(
                                        new Point(testScore, 67.000, Point.CARTESIAN),
                                        new Point(20.000, 54.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230)).build()
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
                new RunCommand(() -> robot.follower.update()), //shouldn't need this cus its called in robot.loop()??? idk ill keep it here fo rnow
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OuttakeIntermediateCommand(robot, true),
                                new FollowPathCommand(robot.follower, paths.get(0))//go to deposit preload
                        ),
                        new SequentialCommandGroup( //deposit preload
                                new HighChamberCommand(robot),
                                new WaitCommand(500),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(200) //dont need this i think?
                        ),
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(1)), //go and intake first sample
                                new SequentialCommandGroup(
                                        new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD),
                                        new IntakeCommand(robot, IntakeArm.IntakeArmState.INTAKE)
                                )
                        ),

                        new SequentialCommandGroup(
                                new TurnCommand(robot.follower, depositSampleAngle), //deposit sample one: 2
                                new IntakeEndCommand(robot, IntakeEnd.ActiveState.REVERSED)
                        ),

                        new WaitCommand(200),
                        new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD), //go turn back and turn intake on
                        new TurnCommand(robot.follower, facingSampleAngle), //turn towards sample two: 3

                        new FollowPathCommand(robot.follower, paths.get(4)),

                        new IntakeCommand(robot, IntakeArm.IntakeArmState.INTAKE),
                        new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD),

                        new SequentialCommandGroup(
                                new TurnCommand(robot.follower, depositSampleAngle), //drop sample two: 5
                                new IntakeEndCommand(robot, IntakeEnd.ActiveState.REVERSED)
                        ),

                        new WaitCommand(200),
                        new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD), //go turn back and turn intake on
                        new TurnCommand(robot.follower, facingSampleAngle), //turn back for sample 3: 6

                        new FollowPathCommand(robot.follower, paths.get(7)), //intake sample 3

                        new SequentialCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(8)), //drop sample 3
                                new IntakeEndCommand(robot, IntakeEnd.ActiveState.REVERSED)
                        ),
                        new WaitCommand(200),
                        new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF),
                        new IntakeRetractCommand(robot, IntakeArm.IntakeArmState.INIT),
                        new GrabOffWallCommand(robot),
                        new WaitCommand(250),

                        new FollowPathCommand(robot.follower, paths.get(9)),


                        new SequentialCommandGroup( //grabbing specimen and preparing to deposit
                                new OuttakeIntermediateCommand(robot),
                                new WaitCommand(150) //WE CAN REMOVE THIS LATER
                                //new WaitCommand(350) //dont need this i think?
                        ),

                        new FollowPathCommand(robot.follower, paths.get(10)), //going to high chamber to deposit specimen 2

                        new SequentialCommandGroup( //depositing specimen 2
                                new HighChamberCommand(robot),
                                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open
                        ),

                        new ParallelCommandGroup( //going back to pick up specimen 3
                                new GrabOffWallCommand(robot), //set outtake up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(11)) //do a lot of things(pushing all samples) and then going to pick up first specimen
                        ),

                        new SequentialCommandGroup( //grabbing specimen and preparing to deposit
                                new WaitCommand(150), //WE CAN REMOVE THIS LATER
                                new OuttakeIntermediateCommand(robot)
                                //new WaitCommand(350) //dont need this i think?
                        ),

                        new FollowPathCommand(robot.follower, paths.get(12)), //going to high chamber to deposit specimen 3

                        new SequentialCommandGroup( //depositing specimen 3
                                new HighChamberCommand(robot),
                                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open
                        ),

                        new ParallelCommandGroup( //going back to pick up specimen 4
                                new GrabOffWallCommand(robot), //set outtake up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(13)) //do a lot of things(pushing all samples) and then going to pick up first specimen
                        ),

                        new SequentialCommandGroup( //grabbing specimen and preparing to deposit
                                new WaitCommand(150), //WE CAN REMOVE THIS LATER
                                new OuttakeIntermediateCommand(robot)
                                //new WaitCommand(350) //dont need this i think?
                        ),

                        new FollowPathCommand(robot.follower, paths.get(14)), //going to high chamber to deposit specimen 4

                        new SequentialCommandGroup( //depositing specimen 4
                                new HighChamberCommand(robot),
                                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open
                        ),

                        new ParallelCommandGroup( //going back to pick up specimen 5
                                new GrabOffWallCommand(robot), //set outtake up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(15)) //do a lot of things(pushing all samples) and then going to pick up first specimen
                        ),

                        new SequentialCommandGroup( //grabbing specimen and preparing to deposit
                                new WaitCommand(150), //WE CAN REMOVE THIS LATER
                                new OuttakeIntermediateCommand(robot)
                                //new WaitCommand(350) //dont need this i think?
                        ),

                        new FollowPathCommand(robot.follower, paths.get(16)), //going to high chamber to deposit specimen 5

                        new SequentialCommandGroup( //depositing specimen 4
                                new HighChamberCommand(robot),
                                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open
                        ),

                        new ParallelCommandGroup(
                                new IntakeCommand(robot, IntakeArm.IntakeArmState.INTAKE), //extending intake to get the IntakeArm in the observation zone for park
                                new FollowPathCommand(robot.follower, paths.get(17)) //park position/location
                        )
                )
        );

        dashboardPoseTracker = robot.follower.getDashboardPoseTracker();
        Drawing.drawRobot(robot.follower.getPose(), "#4CAF50");
        Drawing.sendPacket();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            robot.loop();

            TelemetryUtil.addData("POSE", robot.follower.getPose());
            TelemetryUtil.addData("TIMER", timer.milliseconds());

            TelemetryUtil.update();

            dashboardPoseTracker.update();
            Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
            Drawing.drawRobot(robot.follower.getPose(), "#4CAF50");
            Drawing.sendPacket();
        }
        robot.end();
    }


}


