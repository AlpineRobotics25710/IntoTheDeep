package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;

/**
 * Gets ready to intake a game piece
 */
@Config
public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(Robot robot, double extendoPos) {
        super(
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INIT), // Might need, might not need, we'll see
                new WaitCommand(400),
                new SequentialCommandGroup(
                        new ExtendoCommand(robot, extendoPos),
                        new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTAKE)
                )
        );
    }

    public IntakeCommand(Robot robot) {
        super(
                new SequentialCommandGroup(
                        new ExtendoCommand(robot, Extendo.MAX_LENGTH),
                        new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTAKE)
                )
        );
    }

    public IntakeCommand(Robot robot, IntakeArm.IntakeArmState state) {
        super(
                new SequentialCommandGroup(
                        new ExtendoCommand(robot, Extendo.MAX_LENGTH),
                        new IntakeArmCommand(robot, state)
                )
        );
    }
}
