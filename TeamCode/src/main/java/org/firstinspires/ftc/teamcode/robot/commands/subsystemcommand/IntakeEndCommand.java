package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;

public class IntakeEndCommand extends InstantCommand {
    public IntakeEndCommand(Robot robot, IntakeEnd.ActiveState state){
        super(
                () -> robot.intakeEnd.setState(state)
        );
    }
}
