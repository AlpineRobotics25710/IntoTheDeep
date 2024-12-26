package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;

/**
 * Gets ready to intake a game piece
 */
public class IntakeCommand extends ParallelCommandGroup {
    private final Robot robot = Robot.getInstance();

    public IntakeCommand(){
        addCommands(
                new InstantCommand(() -> robot.intakeClaw.setSwivelState(Claw.SwivelState.INTAKE)),
                new InstantCommand(() -> robot.intakeArm.setState(Arm.ArmState.INTAKE)),
                new InstantCommand(() -> robot.intakeClaw.setClawState(Claw.ClawState.OPEN))
        );
        addRequirements(robot.intakeArm, robot.intakeClaw);
    }
}
