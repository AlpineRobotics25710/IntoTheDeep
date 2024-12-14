package org.firstinspires.ftc.teamcode.robot.control.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;

public class RobotCentricMecanumDrivetrain extends Drivetrain {
    RobotCentricMecanumDrivetrain(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, CustomGamepad gamepad) {
        super(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad);
    }

    @Override
    public void update() {
        double y = -gamepad.getLeftStick().getY() * gamepad.getLeftStick().getSensitivity(); // Y stick value is reversed
        double x = gamepad.getLeftStick().getX() * strafingMultiplier * gamepad.getRightStick().getSensitivity(); // Counteract imperfect strafing
        double rx = gamepad.getRightStick().getY() * gamepad.getRightStick().getSensitivity();

        // Denominator is the largest motor power (absolute value) or 1
        double denominator;
        if (gamepad.getLeftBumper().isPressed()) {
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.75);
        } else {
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        }
        
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}
