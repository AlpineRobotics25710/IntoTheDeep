package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class SwivelCommand extends InstantCommand {
    public SwivelCommand(Robot robot, OuttakeClaw.OuttakeSwivelState state) {
        super(
                () -> robot.outtakeClaw.setSwivelState(state)
        );
    }
}
