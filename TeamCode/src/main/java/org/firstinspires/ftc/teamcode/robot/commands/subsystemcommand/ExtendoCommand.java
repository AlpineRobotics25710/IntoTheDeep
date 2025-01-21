package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;

public class ExtendoCommand extends InstantCommand {
    public ExtendoCommand(Robot robot, double target){
        super(
                () ->  {
                    if (target == Extendo.BASE_POS) {
                        robot.extendoRight.setPower(-0.4);
                    } else if (target == Extendo.MAX_LENGTH) {
                        robot.extendoRight.setPower(0.4);
                    }
                }
        );
    }
}
