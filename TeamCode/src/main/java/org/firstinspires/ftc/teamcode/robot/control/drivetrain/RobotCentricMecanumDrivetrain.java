package org.firstinspires.ftc.teamcode.robot.control.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;

public class RobotCentricMecanumDrivetrain extends Drivetrain {
    RobotCentricMecanumDrivetrain(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, CustomGamepad gamepad, HardwareMap hardwareMap) {
        super(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad, hardwareMap);
    }

    @Override
    public void update() {
        double y = -gamepad.leftStick().getY() * gamepad.leftStick().getMultiplier(); // Y stick value is reversed
        double x = gamepad.leftStick().getX() * strafingMultiplier * gamepad.rightStick().getMultiplier(); // Counteract imperfect strafing
        double rx = gamepad.rightStick().getY() * gamepad.rightStick().getMultiplier();

        // Denominator is the largest motor power (absolute value) or 1
        double denominator;
        if (gamepad.leftBumper().isPressed()) {
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
