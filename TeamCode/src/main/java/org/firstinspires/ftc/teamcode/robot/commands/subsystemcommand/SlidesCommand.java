package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class SlidesCommand extends InstantCommand {
    public SlidesCommand(Robot robot, double target, boolean manualMode) {
        super(() -> {
            if (manualMode) {
                robot.outtakeSlides.setSlidesPower(target);
            } else {
                robot.outtakeSlides.setTargetPosition(target);
            }
        });
    }
}

