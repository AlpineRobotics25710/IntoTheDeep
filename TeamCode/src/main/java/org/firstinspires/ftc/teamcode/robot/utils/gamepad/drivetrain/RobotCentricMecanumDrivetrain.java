package org.firstinspires.ftc.teamcode.robot.utils.gamepad.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class RobotCentricMecanumDrivetrain extends Drivetrain {
    RobotCentricMecanumDrivetrain(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, Gamepad gamepad) {
        super(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad);
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        double y = -gamepad.left_stick_y * sensitivity; // Y stick value is reversed
        double x = gamepad.left_stick_x * strafingMultiplier * 0.75; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x * sensitivity;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if (gamepad.right_bumper) {
            if (y > 0) {
                frontLeftMotor.setPower(0.75);
                backLeftMotor.setPower(0.75);
                frontRightMotor.setPower(0.75);
                backRightMotor.setPower(0.75);
            } else if (y < -0){
                frontLeftMotor.setPower(-0.75);
                backLeftMotor.setPower(-0.75);
                frontRightMotor.setPower(-0.75);
                backRightMotor.setPower(-0.75);
            } else {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }
        } else if (gamepad.left_bumper) {
            if (x > 0) {
                frontLeftMotor.setPower(0.75);
                backLeftMotor.setPower(-0.75);
                frontRightMotor.setPower(-0.75);
                backRightMotor.setPower(0.75);
            } else if (x < -0){
                frontLeftMotor.setPower(-0.75);
                backLeftMotor.setPower(0.75);
                frontRightMotor.setPower(0.75);
                backRightMotor.setPower(-0.75);
            } else {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }
        } else {
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
