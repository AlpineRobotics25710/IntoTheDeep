package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SlidesCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class HighChamberCommand extends ParallelCommandGroup { //subject to change
    public HighChamberCommand(Robot robot) {
        super(
                new SlidesCommand(robot, OuttakeSlides.HIGH_CHAMBER, false),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.HIGH_CHAMBER),
                //new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.BOTTOM)
        );
    }
}
