package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;

/**
 * Gets ready to intake a game piece
 */
@Config
public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(Robot robot, double extendoPos) {
        super(
                new InstantCommand(() -> {  // Might need, might not need, we'll see
                    if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT ||
                            robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.INIT ||
                            robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.TRANSFER
                    ) {
                        robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.GIVE_SPACE_FOR_INTAKE);
                    }
                }),
                new WaitCommand(500),
                new ExtendoCommand(robot, extendoPos)
        );
    }

    public IntakeCommand(Robot robot) {
        super(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {  // Might need, might not need, we'll see
                            if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT ||
                                    robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.INIT ||
                                    robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.TRANSFER
                            ) {
                                robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.GIVE_SPACE_FOR_INTAKE);
                            }
                        }),
                        new WaitCommand(500),
                        new ExtendoCommand(robot, Extendo.MAX_LENGTH),
                        new WaitCommand(700),
                        new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTAKE)
                )
        );
    }

    public IntakeCommand(Robot robot, IntakeArm.IntakeArmState state) {
        super(
                new SequentialCommandGroup(
                        new ExtendoCommand(robot, Extendo.MAX_LENGTH),
                        new WaitCommand(500),
                        new IntakeArmCommand(robot, state)
                )
        );
    }
}
