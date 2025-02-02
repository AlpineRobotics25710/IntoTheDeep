package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeEndCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot) {
        addCommands(
                new IntakeRetractCommand(robot),
                new OuttakeRetractCommand(robot),
                new ParallelCommandGroup(
                        new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.SIDEWAYS),
                        new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                        new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.TRANSFER),
                        new IntakeArmCommand(robot, IntakeArm.IntakeArmState.TRANSFER),
                        new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD)
                )
        );
    }
}
