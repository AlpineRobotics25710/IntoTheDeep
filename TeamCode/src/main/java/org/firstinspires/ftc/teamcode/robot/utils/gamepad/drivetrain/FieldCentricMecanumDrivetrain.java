package org.firstinspires.ftc.teamcode.robot.utils.gamepad.drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricMecanumDrivetrain extends Drivetrain {
    private final IMU imu;
    private RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

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
                                         DcMotor frontRightMotor, DcMotor backRightMotor, Gamepad gamepad, IMU imu,
                                         RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
                                         RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection
    ) {
        super(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad);
        this.imu = imu;
        setRevHubOrientations(logoFacingDirection, usbFacingDirection);
    }

    /**
     * Sets the REV Hub orientation. By default, the logo is set to facing UP and the USB is set to
     * facing FORWARD.
     * @param logoFacingDirection the direction the logo is facing on the REV hub
     * @param usbFacingDirection the direction the USB is facing on the REV hub
     */
    public void setRevHubOrientations(RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection, RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection) {
        this.logoFacingDirection = logoFacingDirection;
        this.usbFacingDirection = usbFacingDirection;
    }

    @Override
    public void init() {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection,
                usbFacingDirection));

        imu.initialize(parameters);
    }

    @Override
    public void update() {
        double y = -gamepad.left_stick_y * sensitivity;
        double x = gamepad.left_stick_x * strafingMultiplier * sensitivity;
        double rx = gamepad.right_stick_x * sensitivity;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad.options) {
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
