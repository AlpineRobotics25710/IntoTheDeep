package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SlidesCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;

@Config
public class HighChamberCommand extends ParallelCommandGroup { //subject to change
    public static double slidesHeight = 0.0;
    public HighChamberCommand(Robot robot, boolean isForward){
        super(
                new SlidesCommand(robot, slidesHeight, false),
                new SequentialCommandGroup(
                        new WaitCommand(0),
                        new OuttakeArmCommand(robot, isForward ? OuttakeArm.OuttakeArmState.SPECIMEN_FRONT : OuttakeArm.OuttakeArmState.SPECIMEN_BACK)
                )
        );
    }
}
