package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class OuttakeIntermediateCommand extends SequentialCommandGroup {
    public static final long SWIVEL_DELAY = 450; // ms

    public OuttakeIntermediateCommand(Robot robot) {
        addCommands(
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE),
                new WaitCommand(SWIVEL_DELAY),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.BOTTOM)
        );
    }
}
