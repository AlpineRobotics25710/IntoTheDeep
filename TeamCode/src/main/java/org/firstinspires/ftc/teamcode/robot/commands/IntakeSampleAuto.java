package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;

public class IntakeSampleAuto extends SequentialCommandGroup {
    public IntakeSampleAuto(Robot robot){
        addCommands(
                new SetIntake(robot, 0, IntakeArm.IntakeArmState.INTAKE, IntakeEnd.ActiveState.FORWARD, false),
                new SetIntake(robot, 0, IntakeArm.IntakeArmState.TRANSFER, IntakeEnd.ActiveState.FORWARD, false)
        );
    }
}
