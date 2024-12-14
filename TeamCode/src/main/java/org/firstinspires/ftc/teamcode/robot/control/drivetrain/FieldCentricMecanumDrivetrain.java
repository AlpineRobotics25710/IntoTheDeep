package org.firstinspires.ftc.teamcode.robot.control.drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;

public class FieldCentricMecanumDrivetrain extends Drivetrain {
    private final IMU imu;

    /**
     * Constructor for FieldCentricMecanumDrivetrain.
     * NOTE: Remember to initialize your wheel motors before calling this constructor. The motors are not
     * initialized by default.
     * @param imu the IMU to use for orientation
     * @param frontLeftMotor the front left wheel motor
     * @param backLeftMotor the back left wheel motor
     * @param frontRightMotor the front right wheel motor
     * @param backRightMotor the back right wheel motor
     */
    FieldCentricMecanumDrivetrain(DcMotor frontLeftMotor, DcMotor backLeftMotor,
                                  DcMotor frontRightMotor, DcMotor backRightMotor, CustomGamepad gamepad, IMU imu,
                                  RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
                                  RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection
    ) {
        super(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad);
        this.imu = imu;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection,
                usbFacingDirection));

        imu.initialize(parameters);
    }

    @Override
    public void update() {
        double y = -gamepad.getLeftStick().getY() * gamepad.getLeftStick().getSensitivity();
        double x = gamepad.getRightStick().getX() * strafingMultiplier * gamepad.getRightStick().getSensitivity();
        double rx = gamepad.getRightStick().getX() * gamepad.getRightStick().getSensitivity();

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad.getOptions().isPressed()) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}
