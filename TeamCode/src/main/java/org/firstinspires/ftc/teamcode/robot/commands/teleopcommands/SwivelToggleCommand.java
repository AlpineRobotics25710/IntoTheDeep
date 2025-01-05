package org.firstinspires.ftc.teamcode.robot.commands.teleopcommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class SwivelToggleCommand extends ConditionalCommand {
    public SwivelToggleCommand(Robot robot) {
        super(
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.WALL_INTAKE),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.TRANSFER),
                () -> robot.outtakeClaw.getSwivelState() == OuttakeClaw.OuttakeSwivelState.TRANSFER
        );
    }
}
