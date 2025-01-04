package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class GrabOffWallCommand extends SequentialCommandGroup {
    public static long slidesTimeout = 500;
    public static long activeTimeout = 500;

    public GrabOffWallCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.GRAB_OFF_WALL)),
                new WaitCommand(slidesTimeout),
                new InstantCommand(() -> robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.WALL_INTAKE)),
                new InstantCommand(() -> robot.intakeEnd.setState(IntakeEnd.ActiveState.FORWARD)),
                new WaitCommand(activeTimeout),
                new InstantCommand(() -> robot.intakeEnd.setState(IntakeEnd.ActiveState.OFF)),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.TRANSFER)),
                        new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.TRANSFER_POS))
                )
        );
    }
}
