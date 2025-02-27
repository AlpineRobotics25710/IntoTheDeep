package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
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
    Robot robot;
    CommandGroupBase deposit;
    CommandGroupBase pickUp;
    CommandGroupBase fullCycle;

    public static int grabX = 10, grabY = 35, scoreX = 40, scoreY = 62;
    Pose grabPose, scorePose;
    PathChain grab, score;
    double initialHeading;
    public void generatePaths() {
        robot.follower.setPose(grabPose);
        grab = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(grabPose.getX(), grabPose.getY()),
                        new Point(scorePose.getX(), scorePose.getY())))
                .setLinearHeadingInterpolation(initialHeading, Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        score = robot.follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(grabPose.getX() - 0.25, grabPose.getY()))) //odo drift
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();
    }

    public InfiniteAutoSpecScoring(Robot robot) {
        this.robot = robot;
        this.initialHeading = robot.follower.getPose().getHeading();

        grabPose = new Pose(grabX, grabY, initialHeading);
        scorePose = new Pose(scoreX, scoreY, Math.toRadians(180));

        generatePaths();
        robot.follower.setPose(grabPose);

        deposit = new SequentialCommandGroup(
                new HighChamberCommand(robot),
                new WaitCommand(DEPOSIT_DELAY),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                new WaitCommand(CLAW_DEPOSIT_DELAY)
        );

        pickUp = new SequentialCommandGroup(
                new OuttakeIntermediateCommand(robot)
        );

        fullCycle = new SequentialCommandGroup(
                new InstantCommand(() -> generatePaths()),
                new ParallelCommandGroup(
                    pickUp,
                    new SequentialCommandGroup(
                        new WaitCommand(200),
                        new FollowPathCommand(robot.follower, score)
                    )
                ),
                deposit,
                new FollowPathCommand(robot.follower, grab)
        );

        addCommands( //will probobly break out of this before it ends
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
        );
    }
    @Override
    public boolean isFinished() {
        if(gamepad1.x){
            robot.follower.breakFollowing();
            robot.follower.startTeleopDrive();
        }
        return gamepad1.x;
    }

}
