package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class TransferCommand extends ParallelCommandGroup {
    public TransferCommand(Robot robot) {
        addCommands(
                new RetractCommand(robot),
                new OuttakeRetractCommand(robot)
        );
    }
}
