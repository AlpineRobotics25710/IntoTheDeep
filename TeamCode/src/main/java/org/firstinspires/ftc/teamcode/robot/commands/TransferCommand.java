package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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

@Config
public class TransferCommand extends SequentialCommandGroup {
    public static final long RETRACT_WAIT = 1000; // Will decrease with tuning

    public TransferCommand(Robot robot) {
        addCommands(
                new IntakeRetractCommand(robot),
                new OuttakeRetractCommand(robot),
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD),
                new WaitCommand(RETRACT_WAIT),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.BASKET_OUTTAKE)
        );
    }
}
