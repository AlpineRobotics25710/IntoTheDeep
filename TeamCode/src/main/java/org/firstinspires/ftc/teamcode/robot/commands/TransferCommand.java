package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.ExtendoSlides;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

public class TransferCommand extends SequentialCommandGroup {
    private static final Robot robot = Robot.getInstance();

    public static final ParallelCommandGroup outtakeTransferCommand = new ParallelCommandGroup(
            new InstantCommand(() -> robot.outtakeClaw.setClawState(Claw.ClawState.OPEN)),
            new InstantCommand(() -> robot.outtakeClaw.setSwivelState(Claw.SwivelState.TRANSFER)),
            new InstantCommand(() -> robot.outtakeArm.setState(Arm.ArmState.TRANSFER)),
            new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.TRANSFER_POS))
    );

    public static final ParallelCommandGroup intakeTransferCommand = new ParallelCommandGroup(
            new InstantCommand(() -> robot.intakeClaw.setClawState(Claw.ClawState.OPEN)),
            new InstantCommand(() -> robot.intakeClaw.setSwivelState(Claw.SwivelState.TRANSFER)),
            new InstantCommand(() -> robot.intakeArm.setState(Arm.ArmState.TRANSFER)),
            new InstantCommand(() -> robot.extendo.setTargetPosition(ExtendoSlides.BASE_POS))
    );

    public TransferCommand(Robot robot) {
        addCommands(
                new ParallelCommandGroup(
                        intakeTransferCommand,
                        outtakeTransferCommand
                ),
                new InstantCommand(() -> robot.outtakeClaw.setClawState(Claw.ClawState.CLOSED)),
                new InstantCommand(() -> robot.intakeClaw.setClawState(Claw.ClawState.OPEN))
        );
        addRequirements(robot.outtakeArm, robot.outtakeClaw, robot.outtakeSlides, robot.intakeArm, robot.intakeClaw, robot.extendo);
    }
}
