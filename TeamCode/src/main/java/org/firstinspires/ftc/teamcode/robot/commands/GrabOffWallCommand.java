package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
public class GrabOffWallCommand extends SequentialCommandGroup {
    public static long SLIDES_DELAY = 0;
    public static long WRIST_DELAY = 200;
    public static long ARM_DELAY = 600;
    public static long ARM_DELAY2 =  200;

    // 🤫🧏
    public GrabOffWallCommand(Robot robot) {
        super(
                new OuttakeSlidesCommand(robot, OuttakeSlides.GRAB_OFF_WALL),
                new ExtendoCommand(robot, Extendo.BASE_POS),
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INIT),
                new WaitCommand(SLIDES_DELAY),
                new InstantCommand(() -> robot.outtakeArm.setWristPosition(OuttakeArm.WRIST_GRAB_OFF_WALL_INTERMEDIATE_POS)),
                new WaitCommand(WRIST_DELAY),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.TOP),
                new InstantCommand(() -> robot.outtakeArm.setArmPosition(0.84)),
                new WaitCommand(ARM_DELAY),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT),
                new WaitCommand(ARM_DELAY2),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN)
        );
    }
}
