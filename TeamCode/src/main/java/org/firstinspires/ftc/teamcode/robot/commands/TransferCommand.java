package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeEndCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.teleopcommands.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot) {
        addCommands(
                new IntakeRetractCommand(robot),
                new OuttakeRetractCommand(robot),
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD),
                new WaitCommand(600),
                new ClawToggleCommand(robot),
                new WaitCommand(250),
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.REVERSED),
                new WaitCommand(150),
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF)
                //new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.HIGH_BASKET_BACK)
        );
    }
}
