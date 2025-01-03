package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SlidesCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;

@Config
public class HighBasketCommand extends ParallelCommandGroup { //subject to change
    public static double slidesHeight = 0.0;

    public HighBasketCommand(Robot robot, boolean facingBasket){
        super(
                new SlidesCommand(robot, slidesHeight),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new OuttakeArmCommand(robot, facingBasket ? OuttakeArm.OuttakeArmState.SAMPLE_FRONT : OuttakeArm.OuttakeArmState.SAMPLE_BACK)
                )
        );
    }
}
