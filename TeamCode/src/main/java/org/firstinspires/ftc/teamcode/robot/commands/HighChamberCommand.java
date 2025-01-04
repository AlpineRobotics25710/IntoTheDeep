package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class HighChamberCommand extends ParallelCommandGroup { //subject to change
    public static long SLIDES_WAIT_TIME = 500;

    public HighChamberCommand(Robot robot, boolean facingChamber) {
        addCommands(
                new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.HIGH_CHAMBER)),
                new WaitCommand(SLIDES_WAIT_TIME),
                new InstantCommand(() -> {
                    if (facingChamber) {
                        robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.SAMPLE_FRONT);
                    } else {
                        robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.SAMPLE_BACK);
                    }
                })
        );
    }
}
