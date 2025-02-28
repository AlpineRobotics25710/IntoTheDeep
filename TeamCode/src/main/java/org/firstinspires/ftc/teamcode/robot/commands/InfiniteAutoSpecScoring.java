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
    Robot robot;
    CommandGroupBase deposit;
    CommandGroupBase pickUp;
    CommandGroupBase fullCycle;

    public static int grabX = 10, grabY = 35, scoreX = 40, scoreY = 62;
    public static Pose grabPose, scorePose;
    PathChain grab, score;
    public static int COUNT = 0;

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
                        new Point(grabPose.getX() - (0.25*COUNT), grabPose.getY()))) //odo drift
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();
    }

    public InfiniteAutoSpecScoring(Robot robot) {
        this.robot = robot;
        grabPose = new Pose(grabX, grabY, Math.toRadians(180));
        scorePose = new Pose(scoreX, scoreY, Math.toRadians(180));

        generatePaths();

        deposit = new SequentialCommandGroup(
                new HighChamberCommand(robot),
                new WaitCommand(DEPOSIT_DELAY),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                new WaitCommand(CLAW_DEPOSIT_DELAY)
        );

        pickUp = new OuttakeIntermediateCommand(robot);

        fullCycle = new SequentialCommandGroup(
                new InstantCommand(this::generatePaths),
                new InstantCommand(() -> COUNT++),
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
                new FollowPathCommand(robot.follower, grab)
        );
//
//        for (int i = 0; i < COUNT; i++) {
//            addCommands(fullCycle);
//        }

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
    public void end(boolean interrupted) {
        if (interrupted) {
            robot.follower.breakFollowing();
            robot.follower.setMaxPower(1);
            robot.follower.startTeleopDrive();
        }
    }


}
