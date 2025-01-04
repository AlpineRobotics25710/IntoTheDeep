package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class HighBasketCommand extends SequentialCommandGroup { //subject to change
    public static long SLIDES_WAIT_TIME = 500;

    public HighBasketCommand(Robot robot, boolean facingBasket) {
        addCommands(
                new InstantCommand(() -> robot.outtakeSlides.setTargetPosition(OuttakeSlides.HIGH_BASKET)),
                new WaitCommand(SLIDES_WAIT_TIME),
                new InstantCommand(() -> {
                    if (facingBasket) {
                        new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.SAMPLE_FRONT);
                    } else {
                        new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.SAMPLE_BACK);
                    }
                })
        );
    }
}
