package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class TransferCommand extends SequentialCommandGroup {
    private Robot robot;

    public TransferCommand(Robot robot) {
        this.robot = robot;
        addRequirements(robot.outtakeArm, robot.outtakeClaw, robot.outtakeSlides, robot.intakeArm, robot.intakeEnd, robot.extendo);
    }

    @Override
    public void initialize() {
        ParallelCommandGroup outtakeTransferCommand = new ParallelCommandGroup(

        );

        ParallelCommandGroup intakeTransferCommand = new ParallelCommandGroup(

        );

        addCommands(
                outtakeTransferCommand,
                intakeTransferCommand
        );
    }
}
