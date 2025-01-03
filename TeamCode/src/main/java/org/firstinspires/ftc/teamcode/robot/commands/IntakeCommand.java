package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;

/**
 * Gets ready to intake a game piece
 */
public class IntakeCommand extends SequentialCommandGroup {
    private final IntakeEnd intakeEnd;
    private final IntakeArm intakeArm;
    private final Extendo extendo;
    private static final long ARM_WAIT_TIME = 750;
    private static final long ACTIVE_WAIT_TIME = 300;

    public IntakeCommand(Robot robot) {
        intakeEnd = robot.intakeEnd;
        intakeArm = robot.intakeArm;
        extendo = robot.extendo;
        addRequirements(robot.intakeArm, robot.extendo);
    }

    @Override
    public void initialize() {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> extendo.setTargetPosition(Extendo.MAX_LENGTH)),
                        new InstantCommand(() -> intakeArm.setState(IntakeArm.IntakeArmState.INTAKE))
                ),
                new WaitCommand(ARM_WAIT_TIME),
                new InstantCommand(() -> intakeEnd.setState(IntakeEnd.ActiveState.FORWARD)),
                new WaitCommand(ACTIVE_WAIT_TIME),
                new ParallelCommandGroup(
                        new InstantCommand(() -> intakeEnd.setState(IntakeEnd.ActiveState.OFF)),
                        new InstantCommand(() -> intakeArm.setState(IntakeArm.IntakeArmState.TRANSFER)),
                        new InstantCommand(() -> extendo.setTargetPosition(Extendo.BASE_POS))
                )
        );
    }
}
