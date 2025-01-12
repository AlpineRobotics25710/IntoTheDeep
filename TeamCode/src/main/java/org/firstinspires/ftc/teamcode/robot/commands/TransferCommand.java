package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot) {
        CommandGroupBase outtakeTransferCommand = new ParallelCommandGroup(
                //new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.TRANSFER_POS)),
                new InstantCommand(() -> robot.outtakeClaw.setClawState(OuttakeClaw.OuttakeClawState.OPEN)),
                new InstantCommand(() -> robot.outtakeClaw.setSwivelState(OuttakeClaw.OuttakeSwivelState.TRANSFER)),
                new InstantCommand(() -> robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.TRANSFER))
        );

        CommandGroupBase intakeTransferCommand = new ParallelCommandGroup(
                new InstantCommand(() -> robot.extendo.setTargetPosition(Extendo.BASE_POS)),
                new InstantCommand(() -> robot.intakeArm.setState(IntakeArm.IntakeArmState.TRANSFER)),
                new InstantCommand(() -> robot.intakeEnd.setState(IntakeEnd.ActiveState.OFF))
        );

        addRequirements(robot.outtakeArm, robot.outtakeClaw, robot.outtakeSlides, robot.intakeArm, robot.intakeEnd, robot.extendo);

        addCommands(
                new ParallelCommandGroup(
                        outtakeTransferCommand,
                        intakeTransferCommand
                ),
                new WaitCommand(200),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED)
        );
    }
}
