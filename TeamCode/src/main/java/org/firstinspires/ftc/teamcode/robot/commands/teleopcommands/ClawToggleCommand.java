package org.firstinspires.ftc.teamcode.robot.commands.teleopcommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class ClawToggleCommand extends ConditionalCommand {
    public ClawToggleCommand(Robot robot) {
        super(
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                () -> robot.outtakeClaw.getClawState() == (OuttakeClaw.OuttakeClawState.OPEN)
        );
    }
}
