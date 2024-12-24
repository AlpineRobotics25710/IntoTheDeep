package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.lib.IntakeArmLib;
import org.firstinspires.ftc.teamcode.robot.lib.IntakeClawLib;
import org.firstinspires.ftc.teamcode.robot.lib.RobotLib;

public class RetractCommand extends ParallelCommandGroup {
    public RetractCommand(RobotLib robot){
        addCommands(
            new InstantCommand(() -> robot.intakeEnd.setSwivelState(IntakeClawLib.SwivelState.FORWARD)),
            new InstantCommand(() -> robot.intakeArm.setState(IntakeArmLib.ArmState.TRANSFER)),
            new InstantCommand(() -> robot.extendo.setExtendoTarget(0))
        );
    }
}
