package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.Robot;
public class BasicTeleOp extends CommandOpMode {
    private Robot robot;
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        driver.getGamepadButton(GamepadKeys.Button.A).whenHeld(
            new InstantCommand(()
        );

        //
        //gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(armIntakeCommand);
        //gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(armTransferCommand);
    }

    @Override
    public void run() {
        double y = Math.pow(-gamepad1.left_stick_y, 3);
        double x = Math.pow(gamepad1.left_stick_x * 1.1, 3);
        double rx = Math.pow(gamepad1.right_stick_x, 3);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        robot.loop();
    }
}
