package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot) {
        addCommands(
                new IntakeRetractCommand(robot),
                new ParallelCommandGroup(
                        new OuttakeRetractCommand(robot),
                        new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.SIDEWAYS),
                        new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN)
                )
        );
    }
}
