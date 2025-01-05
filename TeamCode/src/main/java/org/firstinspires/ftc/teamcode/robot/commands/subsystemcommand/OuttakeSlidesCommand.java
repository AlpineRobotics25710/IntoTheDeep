package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class OuttakeSlidesCommand extends InstantCommand {
    public OuttakeSlidesCommand(Robot robot, double targetPos) {
        super(() -> robot.outtakeSlides.setTargetPosition(targetPos));
    }
}
