package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class LowBasketCommand extends SequentialCommandGroup {
    public static long SLIDES_WAIT_TIME = 500;

    public LowBasketCommand(Robot robot, boolean facingBasket) {
        addCommands(
                new OuttakeSlidesCommand(robot, OuttakeSlides.LOW_BASKET),
                new WaitCommand(SLIDES_WAIT_TIME),
                new InstantCommand(() -> {
                    if (facingBasket) {
                        robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.OUTTAKE_FRONT);
                    } else {
                        robot.outtakeArm.setState(OuttakeArm.OuttakeArmState.OUTTAKE_BACK);
                    }
                })
        );
    }
}
