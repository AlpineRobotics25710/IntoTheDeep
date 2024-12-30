package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class TransferCommand extends SequentialCommandGroup {

    public static final ParallelCommandGroup outtakeTransferCommand = new ParallelCommandGroup(
            /*new InstantCommand(() -> robot.outtakeClaw.setClawState(Claw.ClawState.OPEN)),
            new InstantCommand(() -> robot.outtakeClaw.setSwivelState(Claw.SwivelState.TRANSFER)),
            new InstantCommand(() -> robot.outtakeArm.setState(Arm.ArmState.TRANSFER)),
            new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.TRANSFER_POS))*/
    );

    public static final ParallelCommandGroup intakeTransferCommand = new ParallelCommandGroup(
            /*new InstantCommand(() -> robot.intakeClaw.setClawState(Claw.ClawState.OPEN)),
            new InstantCommand(() -> robot.intakeClaw.setSwivelState(Claw.SwivelState.TRANSFER)),
            new InstantCommand(() -> robot.intakeArm.setState(Arm.ArmState.TRANSFER))*/
       //     new InstantCommand(() -> robot.extendo.setTargetPosition(ExtendoSlides.BASE_POS))
    );

    public TransferCommand(Robot robot) {
        addCommands(
                new ParallelCommandGroup(
                        intakeTransferCommand,
                        outtakeTransferCommand
                )
               // new InstantCommand(() -> robot.outtakeClaw.setClawState(Claw.ClawState.CLOSED)),
                //new InstantCommand(() -> robot.intakeClaw.setClawState(Claw.ClawState.OPEN))
        );
        addRequirements(robot.outtakeArm, robot.outtakeClaw, robot.outtakeSlides, robot.intakeArm, robot.intakeEnd, robot.extendo);
    }
}
