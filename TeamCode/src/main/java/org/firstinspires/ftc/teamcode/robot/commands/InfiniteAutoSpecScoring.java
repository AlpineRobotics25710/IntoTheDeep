package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

@Config
public class InfiniteAutoSpecScoring extends SequentialCommandGroup {
    public static final long CLAW_DEPOSIT_DELAY = 100;
    public static final long DEPOSIT_DELAY = 200;
    public static int grabX = 10, grabY = 35, scoreX = 40, scoreY = 62;
    public static Pose grabPose, scorePose;
    public static int count = 0;
    public static int overallCount = 0;
    Robot robot;
    CommandGroupBase deposit;
    CommandGroupBase pickUp;
    CommandGroupBase fullCycle;
    PathChain grab, score;
    public static final InstantCommand INCREMENT_COUNT = new InstantCommand(() -> count++);

    public InfiniteAutoSpecScoring(Robot robot, Pose grabPose) {
        this.robot = robot;
        InfiniteAutoSpecScoring.grabPose = grabPose;

        /*addCommands( //will probobly break out of this before it ends
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle,
                fullCycle
        );*/
    }

    @Override
    public void execute() {
        for (int i = 0; i < count; i++) {
            addCommands(fullCycle);
        }

        super.execute();
    }

    @Override
    public void initialize() {
        super.initialize();
        //grabPose = new Pose(grabX, grabY, Math.toRadians(180));
        scorePose = new Pose(scoreX, scoreY, Math.toRadians(180));

        generatePaths();
    }

    public void generatePaths() {
        robot.follower.setPose(grabPose);
        grab = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(grabPose.getX(), grabPose.getY()),
                        new Point(scorePose.getX(), scorePose.getY())))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        score = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(grabPose.getX() - (0.25 * overallCount), grabPose.getY()))) //odo drift
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        deposit = new SequentialCommandGroup(
                new HighChamberCommand(robot),
                new WaitCommand(DEPOSIT_DELAY),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                new WaitCommand(CLAW_DEPOSIT_DELAY)
        );

        pickUp = new OuttakeIntermediateCommand(robot);

        fullCycle = new SequentialCommandGroup(
                new WaitCommand(200),
                new ParallelCommandGroup(
                        pickUp,
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new FollowPathCommand(robot.follower, score)
                        )
                ),
                new WaitCommand(400),
                deposit,
                new WaitCommand(400),
                new GrabOffWallCommand(robot),
                new FollowPathCommand(robot.follower, grab),
                new InstantCommand(() -> overallCount++),
                new InstantCommand(this::generatePaths)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robot.follower.breakFollowing();
        robot.follower.setMaxPower(1);
        robot.follower.startTeleopDrive();
        count = 0;
    }
}
