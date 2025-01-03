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
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class HighBasketCommand extends SequentialCommandGroup { //subject to change
    public static long SLIDES_WAIT_TIME = 500;

    public HighBasketCommand(Robot robot, boolean facingBasket){
        addCommands(
                /*new SlidesCommand(robot, slidesHeight),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new OuttakeArmCommand(robot, facingBasket ? OuttakeArm.OuttakeArmState.SAMPLE_FRONT : OuttakeArm.OuttakeArmState.SAMPLE_BACK)
                )*/
                // Why can't we use this instead?
                new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.HIGH_BASKET)),
                new WaitCommand(SLIDES_WAIT_TIME),
                new InstantCommand(() -> robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.SAMPLE_FRONT))
        );
    }
}
