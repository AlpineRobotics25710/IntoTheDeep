package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeEndCommand;
import org.firstinspires.ftc.teamcode.robot.commands.teleopcommands.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot) {
        addCommands(
                new IntakeRetractCommand(robot),
                new OuttakeRetractCommand(robot),
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD),
                new WaitCommand(750),
                new ClawToggleCommand(robot),
                new WaitCommand(250),
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF)
        );
    }
}
