package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Robot robot){
        addCommands(
            new RetractCommand(robot),
            //outtake command needed here slides come down and arm comes into transfer position might make a whole new command for that idk ;/
            new InstantCommand(() -> robot.intakeEnd.setClawState(Claw.ClawState.OPEN))
            //outtake arm goes back up
        );
    }
}
