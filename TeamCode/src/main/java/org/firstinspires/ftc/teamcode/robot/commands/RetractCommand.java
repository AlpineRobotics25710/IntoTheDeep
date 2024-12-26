package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.ExtendoSlides;

/**
 * Closes the claw on a game piece and brings it into the transfer position
 */
public class RetractCommand extends ParallelCommandGroup {
    private final Robot robot = Robot.getInstance();

    public RetractCommand(){
        addCommands(
                new InstantCommand(() -> robot.extendo.setTargetPosition(ExtendoSlides.BASE_POS)),
                new InstantCommand(() -> robot.intakeClaw.setClawState(Claw.ClawState.CLOSED)),
                new InstantCommand(() -> robot.intakeClaw.setSwivelState(Claw.SwivelState.TRANSFER)),
                new InstantCommand(() -> robot.intakeArm.setState(Arm.ArmState.TRANSFER))
        );
    }
}
