package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;

public class IntakeRetractCommand extends SequentialCommandGroup {
    public IntakeRetractCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.outtakeArm.setArmPosition(OuttakeArm.ARM_TRANSFER_POS - 0.1)), // Might need, might not need, we'll see
                new WaitCommand(400),
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.TRANSFER),
                new ExtendoCommand(robot, Extendo.BASE_POS),
                new InstantCommand(() -> robot.outtakeArm.setArmPosition(OuttakeArm.ARM_TRANSFER_POS)) // Might need, might not need, we'll see
        );
    }

    public IntakeRetractCommand(Robot robot, IntakeArm.IntakeArmState state){
        super(
                new InstantCommand(() -> robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.INIT)), // Might need, might not need, we'll see
                new IntakeArmCommand(robot, state),
                new WaitCommand(400),
                new ExtendoCommand(robot, Extendo.BASE_POS)
        );
    }
}
