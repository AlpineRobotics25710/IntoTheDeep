package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo.BASE_POS;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;

public class IntakeRetractCommand extends ParallelCommandGroup {
    public IntakeRetractCommand(Robot robot) {
        super(
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.TRANSFER),
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new ExtendoCommand(robot, BASE_POS)
                )
        );
    }
}
