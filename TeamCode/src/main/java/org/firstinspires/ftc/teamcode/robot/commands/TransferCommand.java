package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

public class TransferCommand extends SequentialCommandGroup {
    private final CommandGroupBase outtakeTransferCommand;
    private final CommandGroupBase intakeTransferCommand;

    public TransferCommand(Robot robot) {
        outtakeTransferCommand = new ParallelCommandGroup(
                new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.TRANSFER_POS)),
                new InstantCommand(() -> robot.outtakeClaw.setClawState(OuttakeClaw.OuttakeClawState.OPEN)),
                new InstantCommand(() -> robot.outtakeClaw.setSwivelState(OuttakeClaw.OuttakeSwivelState.TRANSFER)),
                new InstantCommand(() -> robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.TRANSFER))
        );

        intakeTransferCommand = new ParallelCommandGroup(
                new InstantCommand(() -> robot.extendo.setTargetPosition(Extendo.BASE_POS)),
                new InstantCommand(() -> robot.intakeArm.setState(IntakeArm.IntakeArmState.TRANSFER)),
                new InstantCommand(() -> robot.intakeEnd.setState(IntakeEnd.ActiveState.OFF))
        );

        addRequirements(robot.outtakeArm, robot.outtakeClaw, robot.outtakeSlides, robot.intakeArm, robot.intakeEnd, robot.extendo);
    }

    @Override
    public void initialize() {
        addCommands(
                outtakeTransferCommand,
                intakeTransferCommand
        );
    }
}
