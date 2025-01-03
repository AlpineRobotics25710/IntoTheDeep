package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class OuttakeClawCommand extends InstantCommand {
    public OuttakeClawCommand(Robot robot, OuttakeClaw.OuttakeClawState state){
        super(
                () -> robot.outtakeClaw.setClawState(state)
        );
    }

}
