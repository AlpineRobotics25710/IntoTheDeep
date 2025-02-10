package org.firstinspires.ftc.teamcode.robot.commands;

import static org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo.TRANSFER_POS;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class OuttakeRetractCommand extends ParallelCommandGroup {
    public OuttakeRetractCommand(Robot robot) {
        super(
                new OuttakeSlidesCommand(robot, TRANSFER_POS),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.TRANSFER),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.SIDEWAYS),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN)
        );
    }

    public OuttakeRetractCommand(Robot robot, int pos) {
        super(
                new OuttakeSlidesCommand(robot, pos),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.TRANSFER),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.SIDEWAYS),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN)
        );
    }
}
