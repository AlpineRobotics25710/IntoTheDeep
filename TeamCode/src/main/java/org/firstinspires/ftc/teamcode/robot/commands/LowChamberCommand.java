package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SlidesCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class LowChamberCommand extends ParallelCommandGroup {
    public static long SLIDES_WAIT_TIME = 500;

    public LowChamberCommand(Robot robot) {
        addCommands(
                new SlidesCommand(robot, OuttakeSlides.LOW_CHAMBER, false),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.SUBMERSIBLE_OUTTAKE_BACK),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.BOTTOM)
        );
    }
}
