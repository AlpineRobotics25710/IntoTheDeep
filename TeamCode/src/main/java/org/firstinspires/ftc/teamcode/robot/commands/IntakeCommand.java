package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;

/**
 * Gets ready to intake a game piece
 */
public class IntakeCommand extends ParallelCommandGroup {
    private final IntakeArm intakeArm;
    private final Extendo extendo;

    public IntakeCommand(IntakeArm intakeArm, Extendo extendo) {
        this.intakeArm = intakeArm;
        this.extendo = extendo;
        addRequirements(intakeArm, extendo);
    }

    @Override
    public void initialize() {
        addCommands(
                new InstantCommand(() -> extendo.setTargetPosition(Extendo.MAX_LENGTH)),
                new InstantCommand(() -> intakeArm.setState(IntakeArm.IntakeArmState.INTAKE))
        );
    }
}
