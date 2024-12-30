package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;

/**
 * Gets ready to intake a game piece
 */
public class IntakeCommand extends ParallelCommandGroup {
    private final IntakeArm intakeArm;
    private final IntakeEnd intakeEnd;

    public IntakeCommand(IntakeArm intakeArm, IntakeEnd intakeEnd) {
        this.intakeArm = intakeArm;
        this.intakeEnd = intakeEnd;
    }

    @Override
    public void initialize() {
        addCommands(
                new InstantCommand(() -> intakeArm.setState(IntakeArm.IntakeArmState.INTAKE)),
                new InstantCommand(() -> intakeEnd.setState(IntakeEnd.ActiveState.ON))
        );
        addRequirements(intakeArm);
    }
}
