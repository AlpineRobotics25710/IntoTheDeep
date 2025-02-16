package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm.WRIST_OUTTAKE_BACK_POS;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class OuttakeIntermediateCommand extends SequentialCommandGroup {
    public static final long CLAW_DELAY = 200; //ms
    public static final long WRIST_DELAY = 300; //ms
    public static final long ARM_DELAY = 100; //ms

    public OuttakeIntermediateCommand(Robot robot) {
        addCommands(
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED),
                new WaitCommand(CLAW_DELAY),
                new InstantCommand(() -> robot.outtakeArm.setWristPosition(0.17)),
                new WaitCommand(WRIST_DELAY),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE),
                new WaitCommand(ARM_DELAY),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.BOTTOM),
                new InstantCommand(() -> robot.outtakeArm.setWristPosition(WRIST_OUTTAKE_BACK_POS))
        );
    }

    public OuttakeIntermediateCommand(Robot robot, boolean lol) {
        addCommands(
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED),

                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE),
                //new WaitCommand(ARM_DELAY),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.BOTTOM),
                new InstantCommand(() -> robot.outtakeArm.setWristPosition(WRIST_OUTTAKE_BACK_POS))
        );
    }
}
