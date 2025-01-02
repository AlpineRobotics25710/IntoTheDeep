package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;

/**
 * Gets ready to intake a game piece
 */
public class IntakeCommand extends ParallelCommandGroup {
    private final Robot robot;

    public IntakeCommand(Robot robot) {
        this.robot = robot;
        addRequirements(robot.intakeArm, robot.extendo);
    }

    @Override
    public void initialize() {
        addCommands(
                new InstantCommand(() -> robot.extendo.setTargetPosition(Extendo.MAX_LENGTH)),
                new InstantCommand(() -> robot.intakeArm.setState(IntakeArm.IntakeArmState.INTAKE))
        );
    }
}
