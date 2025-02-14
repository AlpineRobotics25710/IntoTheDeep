package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class GrabOffWallCommand extends SequentialCommandGroup {
    public static long SLIDES_DELAY = 800;
    public static long WRIST_DELAY = 250;
    public static long ARM_DELAY1 = 750;
    public static long ARM_DELAY2 = 300;

    // 🤫🧏
    public GrabOffWallCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.outtakeArm.setWristPosition(OuttakeArm.WRIST_GRAB_OFF_WALL_INTERMEDIATE_POS)),
                new WaitCommand(WRIST_DELAY),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.TOP),
                new InstantCommand(() -> robot.outtakeArm.setArmPosition(0.73)),
                new WaitCommand(ARM_DELAY1),
                new OuttakeSlidesCommand(robot, OuttakeSlides.GRAB_OFF_WALL),
                new ExtendoCommand(robot, Extendo.BASE_POS),
                new InstantCommand(() -> {
                    if (robot.outtakeSlides.getTargetPosition() > OuttakeSlides.GRAB_OFF_WALL || robot.extendo.getTargetPosition() > Extendo.BASE_POS) {
                        new WaitCommand(SLIDES_DELAY);
                    }
                }),
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INIT),
                new InstantCommand(() -> {
                    if (robot.intakeArm.currentState != IntakeArm.IntakeArmState.INIT) {
                        new WaitCommand(300);
                    }
                }),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT),
                new WaitCommand(ARM_DELAY2)
        );
    }
}
