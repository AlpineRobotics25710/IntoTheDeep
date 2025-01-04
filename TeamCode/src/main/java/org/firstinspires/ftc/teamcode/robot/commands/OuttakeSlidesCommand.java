package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class OuttakeSlidesCommand extends InstantCommand {
    public OuttakeSlidesCommand(Robot robot, double target) {
        super(
                () -> robot.outtakeSlides.setTargetPosition(target)
        );
    }
}
