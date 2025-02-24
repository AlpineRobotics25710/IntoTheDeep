package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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
    public static long INTAKE_ARM_DELAY = 400;
    public static long EXTENDO_DELAY = 500;
    public static long SLIDES_DELAY = 800;
    public static long WRIST_DELAY = 250;
    public static long ARM_DELAY1 = 750;
    public static long ARM_DELAY2 = 300;
    public static double OUTTAKE_SLOW_DOWN_POS = 0.73; // idk what else to call it

    // 🤫🧏
    public GrabOffWallCommand(Robot robot) {
        // Changed the instant commands that do the wait commands if a mechanism is not in certain position into conditional command
        // IF THEY DON'T WORK I PUT THE OLD CODE AT THE BOTTOM JUST COPY PASTE THAT HERE DO NOT REVERT I REPEAT DO NOT REVERT THE COMMIT BECAUSE I WILL FIND YOU
        super(
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INIT),
                new ConditionalCommand(new WaitCommand(INTAKE_ARM_DELAY), new InstantCommand(), () -> robot.intakeArm.currentState == IntakeArm.IntakeArmState.INIT),
                new ExtendoCommand(robot, Extendo.BASE_POS),
                new ConditionalCommand(new WaitCommand(EXTENDO_DELAY), new InstantCommand(), () -> robot.extendo.getTargetPosition() > Extendo.BASE_POS),
                new InstantCommand(() -> robot.outtakeArm.setWristPosition(OuttakeArm.WRIST_GRAB_OFF_WALL_INTERMEDIATE_POS)),
                new WaitCommand(WRIST_DELAY),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.TOP),
                new InstantCommand(() -> robot.outtakeArm.setArmPosition(OUTTAKE_SLOW_DOWN_POS)),
                new WaitCommand(ARM_DELAY1),
                new OuttakeSlidesCommand(robot, OuttakeSlides.GRAB_OFF_WALL),
                new ConditionalCommand(new WaitCommand(SLIDES_DELAY), new InstantCommand(), () -> robot.outtakeSlides.getTargetPosition() > OuttakeSlides.GRAB_OFF_WALL),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT),
                new WaitCommand(ARM_DELAY2)
        );
    }

    /*
    super(
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INIT),
                new InstantCommand(() -> {
                    if (robot.intakeArm.currentState != IntakeArm.IntakeArmState.INIT) {
                        new WaitCommand(INTAKE_ARM_DELAY);
                    }
                }),
                new ExtendoCommand(robot, Extendo.BASE_POS),
                new InstantCommand(() -> {
                    if (robot.extendo.getTargetPosition() > Extendo.BASE_POS) {
                        new WaitCommand(EXTENDO_DELAY);
                    }
                }),
                new InstantCommand(() -> robot.outtakeArm.setWristPosition(OuttakeArm.WRIST_GRAB_OFF_WALL_INTERMEDIATE_POS)),
                new WaitCommand(WRIST_DELAY),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.TOP),
                new InstantCommand(() -> robot.outtakeArm.setArmPosition(OUTTAKE_SLOW_DOWN_POS)),
                new WaitCommand(ARM_DELAY1),
                new OuttakeSlidesCommand(robot, OuttakeSlides.GRAB_OFF_WALL),
                new InstantCommand(() -> {
                    if (robot.outtakeSlides.getTargetPosition() > OuttakeSlides.GRAB_OFF_WALL) {
                        new WaitCommand(SLIDES_DELAY);
                    }
                }),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT),
                new WaitCommand(ARM_DELAY2)
        );
     */
}
