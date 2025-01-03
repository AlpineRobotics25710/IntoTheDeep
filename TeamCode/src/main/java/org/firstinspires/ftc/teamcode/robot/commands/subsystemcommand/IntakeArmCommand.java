package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;

public class IntakeArmCommand extends InstantCommand {
    public IntakeArmCommand(Robot robot, IntakeArm.IntakeArmState armState){
        super(
                () -> robot.intakeArm.setState(armState)
        );
    }
}
