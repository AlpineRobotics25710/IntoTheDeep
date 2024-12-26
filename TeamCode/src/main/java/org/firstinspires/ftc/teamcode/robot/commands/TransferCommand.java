package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

public class TransferCommand extends SequentialCommandGroup {
    private final Robot robot = Robot.getInstance();

    private final ParallelCommandGroup outtakeTransferCommand = new ParallelCommandGroup(
            new InstantCommand(() -> robot.outtakeClaw.setClawState(Claw.ClawState.OPEN)),
            new InstantCommand(() -> robot.outtakeClaw.setSwivelState(Claw.SwivelState.TRANSFER)),
            new InstantCommand(() -> robot.outtakeArm.setState(Arm.ArmState.TRANSFER)),
            new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.TRANSFER_POS))
    );

    private final ParallelCommandGroup intakeTransferCommand = new ParallelCommandGroup(
            // Retract command should probably be called manually but we can add it here as well just in case
            new RetractCommand(),
            new InstantCommand(() -> robot.intakeClaw.setClawState(Claw.ClawState.OPEN)),
            new InstantCommand(() -> robot.intakeClaw.setSwivelState(Claw.SwivelState.TRANSFER)),
            new InstantCommand(() -> robot.intakeArm.setState(Arm.ArmState.TRANSFER))
    );

    public TransferCommand(Robot robot) {
        addCommands(
                intakeTransferCommand,
                outtakeTransferCommand,
                new InstantCommand(() -> robot.outtakeClaw.setClawState(Claw.ClawState.CLOSED)),
                new InstantCommand(() -> robot.intakeClaw.setClawState(Claw.ClawState.OPEN))
        );
    }
}
