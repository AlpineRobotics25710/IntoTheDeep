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

    public static double facingSampleAngle = 300;
    public static double depositSampleAngle = 225;

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
                        .addPath( //line 1
                                new BezierLine(
                                        new Point(8.500, 65.500, Point.CARTESIAN),
                                        new Point(38.500, 77.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build()
        );


        paths.add( //index 1
                robot.follower.pathBuilder()
                        .addPath( //line 2
                                new BezierLine(
                                        new Point(38.500, 77.000, Point.CARTESIAN),
                                        new Point(30.000, 50.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(facingSampleAngle))
                        .build()
        );

        //EXTEND INTAKE

        //TURN - dont call this in code, doesn't work so skip this index and use turn command
        paths.add( //index 2
                robot.follower.pathBuilder()
                        .addPath( //line 3
                                new BezierLine(
                                        new Point(30.000, 50.000, Point.CARTESIAN),
                                        new Point(30.000, 50.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(facingSampleAngle), Math.toRadians(depositSampleAngle))
                        .build()
        );

        paths.add( //index 3
                robot.follower.pathBuilder()
                        .addPath( //line 4
                                new BezierLine(
                                        new Point(30.000, 50.000, Point.CARTESIAN),
                                        new Point(30.000, 40.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(depositSampleAngle), Math.toRadians(facingSampleAngle))
                        .build()
        );

        //TURN - dont call this in code, doesn't work so skip this index and use turn command
        paths.add( //index 4
                robot.follower.pathBuilder()
                        .addPath( //line 5
                                new BezierLine(
                                        new Point(30.000, 40.000, Point.CARTESIAN),
                                        new Point(30.000, 40.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(facingSampleAngle), Math.toRadians(depositSampleAngle))
                        .build()
        );
        //INTAKE

        //TURN
        paths.add( //index 5
                robot.follower.pathBuilder()
                        .addPath( //line 6
                                new BezierCurve(
                                        new Point(30.000, 40.000, Point.CARTESIAN),
                                        new Point(34.000, 36.000, Point.CARTESIAN),
                                        new Point(54.000, 36.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(270))
                        .build()
        );

        paths.add( //index 6
                robot.follower.pathBuilder()
                        .addPath( //line 7
                                new BezierLine(
                                        new Point(54.000, 36.000, Point.CARTESIAN),
                                        new Point(12.000, 36.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(270))
                        .build()
        );

        paths.add( //index 7
                robot.follower.pathBuilder()
                        .addPath( //line 8
                                new BezierCurve(
                                        new Point(12.000, 36.000, Point.CARTESIAN),
                                        new Point(38.000, 36.000, Point.CARTESIAN),
                                        new Point(9.000, 36.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                        .build()
        );

        paths.add( //index 8
                robot.follower.pathBuilder()
                        .addPath( //line 9
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
                        .addPath( //line 14
                                new BezierCurve(
                                        new Point(38.500, 69.500, Point.CARTESIAN),
                                        new Point(20.500, 64.500, Point.CARTESIAN),
                                        new Point(30.000, 36.000, Point.CARTESIAN),
                                        new Point(9.000, 36.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 14
                robot.follower.pathBuilder()
                        .addPath( //line 15
                                new BezierLine(
                                        new Point(9.000, 36.000, Point.CARTESIAN),
                                        new Point(38.500, 67.000, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );

        paths.add( //index 15
                robot.follower.pathBuilder()
                        .addPath( //line 16
                                new BezierLine(
                                        new Point(38.500, 67.000, Point.CARTESIAN),
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
        robot = new Robot(hardwareMap, true, false);
        generatePath();

        CommandScheduler.getInstance().schedule(
                new RunCommand(() -> robot.follower.update()), //shouldn't need this cus its called in robot.loop()??? idk ill keep it here for now
                new SequentialCommandGroup(
                        new ParallelCommandGroup( //moving to deposit
                                new OuttakeIntermediateCommand(robot, true),
                                new FollowPathCommand(robot.follower, paths.get(0))//go to deposit preload (specimen 1)
                        ),
                        new SequentialCommandGroup( //moving arm to deposit preload
                                new HighChamberCommand(robot),
                                new WaitCommand(250),
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(200) //dont need this i think?
                        ),

                        new ParallelCommandGroup( //going to push samples
                                new FollowPathCommand(robot.follower, paths.get(1)), //go to position to push sample 1
                                new IntakeCommand(robot, IntakeArm.IntakeArmState.INTERIM)
                        ),

                        new IntakeCommand(robot, IntakeArm.IntakeArmState.INTAKE), //need to change this to state to sweep(intake state is a temporary placeholder)
                        //TODO: add intake arm sweeping state and get values for it and change it in path
                        new WaitCommand(50),

                        new TurnCommand(robot.follower, depositSampleAngle), //pushed/pushing sample one by turning: ignore index 2 in arraylist of pathchains

                        new ParallelCommandGroup(//going to position to push sample 2
                                new FollowPathCommand(robot.follower, paths.get(3)), //go to position to push sample 2
                                new IntakeCommand(robot, IntakeArm.IntakeArmState.INTERIM)
                        ),

                        new IntakeCommand(robot, IntakeArm.IntakeArmState.INTAKE), //need to change this to state to sweep(intake state is a temporary placeholder)
                        //TODO: add intake arm sweeping state and get values for it and change it in path
                        new WaitCommand(50),

                        new TurnCommand(robot.follower, depositSampleAngle), //pushed/pushing sample one by turning: ignore index 4 in arraylist of pathchains

                        new ParallelCommandGroup(//going to position to push sample 2
                                new FollowPathCommand(robot.follower, paths.get(5)), //go to position to push sample 3
                                new IntakeCommand(robot, IntakeArm.IntakeArmState.INTERIM)
                        ),

                        new IntakeCommand(robot, IntakeArm.IntakeArmState.INTAKE), //need to change this to state to sweep(intake state is a temporary placeholder)
                        //TODO: add intake arm sweeping state and get values for it and change it in path
                        new WaitCommand(50),

                        new FollowPathCommand(robot.follower, paths.get(6)), //pushing sample 3

                        new ParallelCommandGroup( //getting in position for specimen pick up
                                new FollowPathCommand(robot.follower, paths.get(7)), //going to wall for specimen 1 pick up
                                new SequentialCommandGroup(
                                    new IntakeRetractCommand(robot, IntakeArm.IntakeArmState.INIT), //retracting intake
                                    new WaitCommand(250), //waiting for intake to retract
                                    new GrabOffWallCommand(robot) //getting ready to pick up specimen 2 off wall
                                )
                        ),

                        new SequentialCommandGroup( //grabbing specimen 2 and preparing to deposit
                                new WaitCommand(150), //giving human player time to adjust
                                new OuttakeIntermediateCommand(robot),
                                new WaitCommand(250) //WE CAN REMOVE THIS LATER
                        ),

                        new FollowPathCommand(robot.follower, paths.get(8)), //going to deposit specimen 2

                        new SequentialCommandGroup( //depositing specimen 2
                                new HighChamberCommand(robot),
                                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open
                        ),

                        new ParallelCommandGroup( //going to pick up specimen 3
                                new GrabOffWallCommand(robot), //set outtake up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(9)) //do a lot of things(pushing all samples) and then going to pick up first specimen
                        ),

                        new SequentialCommandGroup( //grabbing specimen 3 and preparing to deposit
                                new WaitCommand(150), //giving human player time to adjust
                                new OuttakeIntermediateCommand(robot),
                                new WaitCommand(250)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(10)), //going to high chamber to deposit specimen 3

                        new SequentialCommandGroup( //depositing specimen 3
                                new HighChamberCommand(robot),
                                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open
                        ),

                        new ParallelCommandGroup( //going back to pick up specimen 4
                                new GrabOffWallCommand(robot), //set outtake up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(11)) //going to pick up specimen 3
                        ),

                        new SequentialCommandGroup( //grabbing specimen 4 and preparing to deposit
                                new WaitCommand(150), //WE CAN REMOVE THIS LATER
                                new OuttakeIntermediateCommand(robot),
                                new WaitCommand(250)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(12)), //going to high chamber to deposit specimen 4

                        new SequentialCommandGroup( //depositing specimen 4
                                new HighChamberCommand(robot),
                                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open
                        ),

                        new ParallelCommandGroup( //going back to pick up specimen 5
                                new GrabOffWallCommand(robot), //set outtake up for next spec pickup
                                new FollowPathCommand(robot.follower, paths.get(13)) //going to pick up specimen 5
                        ),

                        new SequentialCommandGroup( //grabbing specimen 5 and preparing to deposit
                                new WaitCommand(150), //WE CAN REMOVE THIS LATER
                                new OuttakeIntermediateCommand(robot),
                                new WaitCommand(250)
                        ),

                        new FollowPathCommand(robot.follower, paths.get(14)), //going to high chamber to deposit specimen 5

                        new SequentialCommandGroup( //depositing specimen 5
                                new HighChamberCommand(robot),
                                new WaitCommand(DEPOSIT_DELAY), //waiting for arm to deposit
                                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                                new WaitCommand(CLAW_DEPOSIT_DELAY) //waiting for claw to open
                        ),

                        new ParallelCommandGroup( //parking
                                new IntakeCommand(robot, IntakeArm.IntakeArmState.INTAKE), //extending intake to get the IntakeArm in the observation zone for park
                                new FollowPathCommand(robot.follower, paths.get(15)) //park position/location
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


