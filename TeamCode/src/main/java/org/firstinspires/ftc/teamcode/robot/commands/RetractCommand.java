package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;

public class RetractCommand extends ParallelCommandGroup {
    public RetractCommand(Robot robot){
        addCommands(
            new InstantCommand(() -> robot.intakeEnd.setSwivelState(Claw.SwivelState.INTAKE)),
            new InstantCommand(() -> robot.intakeArm.setState(Arm.ArmState.TRANSFER)),
            new InstantCommand(() -> robot.extendo.setTargetPosition(0))
        );
    }
}
