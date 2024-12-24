package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.lib.IntakeClawLib;
import org.firstinspires.ftc.teamcode.robot.lib.RobotLib;
import org.firstinspires.ftc.teamcode.robot.lib.IntakeArmLib;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@TeleOp(group="test")
public class LibTeleOp extends CommandOpMode {
    private GamepadEx gp1;
    private final RobotLib robot = RobotLib.getInstance();

    @Override
    public void initialize() {
        super.reset();
        robot.init(hardwareMap);
        register(robot.intakeArm, robot.intakeEnd);
        gp1 = new GamepadEx(gamepad1);
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> robot.intakeArm.setState(IntakeArmLib.ArmState.INTAKE))
        );
        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> robot.intakeArm.setState(IntakeArmLib.ArmState.TRANSFER))
        );
        gp1.getGamepadButton(GamepadKeys.Button.X).whenHeld(
                new InstantCommand(() -> robot.intakeEnd.setSwivelState(IntakeClawLib.SwivelState.FORWARD))
        );
        gp1.getGamepadButton(GamepadKeys.Button.X).whenReleased(
                new InstantCommand(() -> robot.intakeEnd.setSwivelState(IntakeClawLib.SwivelState.BACKWARD))
        );
    }

    @Override
    public void run() {
//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x * 1.1;
//        double rx = gamepad1.right_stick_x;
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        robot.frontLeftMotor.setPower(frontLeftPower);
//        robot.backLeftMotor.setPower(backLeftPower);
//        robot.frontRightMotor.setPower(frontRightPower);
//        robot.backRightMotor.setPower(backRightPower);

        super.run();

        TelemetryUtil.packet.put("Current Arm State", robot.intakeArm.currentState);
        TelemetryUtil.sendTelemetry();
    }
}
