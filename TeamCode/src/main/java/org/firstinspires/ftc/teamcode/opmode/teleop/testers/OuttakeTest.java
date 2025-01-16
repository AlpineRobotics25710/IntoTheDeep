package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
@TeleOp
public class OuttakeTest extends LinearOpMode {
    public void runOpMode() {
        TelemetryUtil.setup(telemetry);
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, false, true);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        // Test outtakeClaw
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED)
        );

        // Test swivel
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.VERTICAL)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.HORIZONTAL)
        );

        // Test outtakeArm
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT)
        );

        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.WALL_INTAKE_BACK)
        );

        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.DEPOSIT_SAMPLE)
        );

        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.TRANSFER)
        );

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            robot.loop();
            telemetry.addData("Claw State", robot.outtakeClaw.getClawState());
            telemetry.addData("Swivel State", robot.outtakeClaw.getSwivelState());
            telemetry.addData("Arm State", robot.outtakeArm.getCurrentState());
            telemetry.update();
        }
    }
}
