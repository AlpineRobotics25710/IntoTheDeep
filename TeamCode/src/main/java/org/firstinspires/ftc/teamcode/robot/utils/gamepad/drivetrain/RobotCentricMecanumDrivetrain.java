package org.firstinspires.ftc.teamcode.robot.utils.gamepad.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.utils.gamepad.CustomGamepad;

public class RobotCentricMecanumDrivetrain extends Drivetrain {
    RobotCentricMecanumDrivetrain(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, CustomGamepad gamepad) {
        super(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad);
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        double y = -gamepad.getLeftStick().getY() * gamepad.getLeftStick().getSensitivity(); // Y stick value is reversed
        double x = gamepad.getLeftStick().getX() * strafingMultiplier * 0.75; // Counteract imperfect strafing
        double rx = gamepad.getRightStick().getY() * gamepad.getRightStick().getSensitivity();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if (gamepad.getRightBumper().isClicked()) {
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
        } else if (gamepad.getLeftBumper().isClicked()) {
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
