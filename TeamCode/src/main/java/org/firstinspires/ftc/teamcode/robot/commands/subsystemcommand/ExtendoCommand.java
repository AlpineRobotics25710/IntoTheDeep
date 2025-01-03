package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;

public class ExtendoCommand extends InstantCommand {
    public ExtendoCommand(Robot robot, int target){
        super(
                () ->  robot.extendo.setTargetPosition(target)
        );
    }
}
