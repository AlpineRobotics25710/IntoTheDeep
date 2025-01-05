package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
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

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN)
        );

        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED)
        );

        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.OUTTAKE)
        );

        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.TRANSFER)
        );

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            robot.loop();
            telemetry.addData("Claw State", robot.outtakeClaw.getClawState());
            telemetry.addData("Swivel State", robot.outtakeClaw.getSwivelState());
            telemetry.update();
        }
    }
}
