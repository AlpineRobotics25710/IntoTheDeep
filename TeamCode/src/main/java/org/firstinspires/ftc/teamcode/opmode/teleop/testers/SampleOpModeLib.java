package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;

/**
 * A small sample op mode that uses FTC Lib commands to set states of the intake mechanism
 */
@TeleOp
public class SampleOpModeLib extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, false, true);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        // Using custom InstantCommands
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> robot.intakeArm.setState(IntakeArm.IntakeArmState.INTAKE)));
        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> robot.intakeArm.setState(IntakeArm.IntakeArmState.TRANSFER)));
   //     gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> robot.intakeClaw.setClawState(ClawState.OPEN)));
     //   gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> robot.intakeClaw.setClawState(ClawState.CLOSED)));
        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> robot.intakeEnd.toggleState()));

        // Using the IntakeCommand
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeCommand(robot.intakeArm, robot.intakeEnd));

        waitForStart();

        while (opModeIsActive()) {
            robot.loop();
        }
    }
}
