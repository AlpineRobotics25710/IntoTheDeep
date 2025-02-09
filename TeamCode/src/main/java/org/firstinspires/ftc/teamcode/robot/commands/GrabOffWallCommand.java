package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class GrabOffWallCommand extends SequentialCommandGroup {
    public static long slidesTimeout = 0;

    // 🤫🧏
    public GrabOffWallCommand(Robot robot) {
        super(
                new OuttakeSlidesCommand(robot, OuttakeSlides.GRAB_OFF_WALL),
                new WaitCommand(slidesTimeout),
                new InstantCommand(() -> robot.outtakeArm.setWristPosition(OuttakeArm.WRIST_GRAB_OFF_WALL_INTERMEDIATE_POS)),
                new WaitCommand(200),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.TOP),
                new InstantCommand(() -> robot.outtakeArm.setArmPosition(0.83)),
                new WaitCommand(900),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT),
                new WaitCommand(200),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN)
        );
    }
}
