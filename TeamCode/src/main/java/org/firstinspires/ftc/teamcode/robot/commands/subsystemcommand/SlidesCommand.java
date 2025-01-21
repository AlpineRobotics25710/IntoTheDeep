package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class SlidesCommand extends InstantCommand {
    public SlidesCommand(Robot robot, double targetOrPower, boolean manualMode) {
        super(() -> {
            if (manualMode) {
                robot.outtakeSlides.setSlidesPower(targetOrPower);
            } else {
                robot.outtakeSlides.setTargetPosition(targetOrPower);
            }
        });
    }
}

