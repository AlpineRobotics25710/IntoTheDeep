package org.firstinspires.ftc.teamcode.robot.control.drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;

/**
 * A builder class for creating different types of drivetrains used in FIRST Tech Challenge robots.
 * This class supports both field-centric and robot-centric mecanum drivetrains.
 * <p>
 * It provides methods to configure essential drivetrain components, including motors, IMU,
 * gamepad, and REV Hub orientation. The type of drivetrain can be set to either
 * {@link DrivetrainType#FIELD_CENTRIC_MECANUM} or {@link DrivetrainType#ROBOT_CENTRIC_MECANUM}.
 */
public class DrivetrainBuilder {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private CustomGamepad gamepad;
    private IMU imu;
    private RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private DrivetrainType type;
    private Follower follower;
    private double strafingMultiplier;

    /**
     * Sets the type of drivetrain to build.
     *
     * @param type The desired drivetrain type (field-centric or robot-centric).
     * @return The current DrivetrainBuilder instance for method chaining.
     */
    public DrivetrainBuilder setType(DrivetrainType type) {
        this.type = type;
        return this;
    }

    /**
     * Sets the motors for the drivetrain.
     *
     * @param frontLeft  The front-left motor.
     * @param backLeft   The back-left motor.
     * @param frontRight The front-right motor.
     * @param backRight  The back-right motor.
     * @return The current DrivetrainBuilder instance for method chaining.
     */
    public DrivetrainBuilder setMotors(DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight) {
        this.frontLeftMotor = frontLeft;
        this.backLeftMotor = backLeft;
        this.frontRightMotor = frontRight;
        this.backRightMotor = backRight;
        return this;
    }

    public DrivetrainBuilder setFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    /**
     * Sets the gamepad to control the drivetrain.
     *
     * @param gamepad The gamepad used for input control.
     * @return The current DrivetrainBuilder instance for method chaining.
     */
    public DrivetrainBuilder setGamepad(CustomGamepad gamepad) {
        this.gamepad = gamepad;
        return this;
    }

    /**
     * Sets the gamepad to control the drivetrain.
     *
     * @param gamepad The gamepad used for input control.
     * @return The current DrivetrainBuilder instance for method chaining.
     */
    public DrivetrainBuilder setGamepad(Gamepad gamepad) {
        this.gamepad = new CustomGamepad(gamepad);
        return this;
    }

    /**
     * Sets the IMU for the drivetrain. Required for field-centric driving.
     *
     * @param imu The inertial measurement unit (IMU) to be used.
     * @return The current DrivetrainBuilder instance for method chaining.
     */
    public DrivetrainBuilder setIMU(IMU imu) {
        this.imu = imu;
        return this;
    }

    /**
     * Sets the orientation of the REV Hub.
     *
     * @param logoFacingDirection The direction the REV Hub logo is facing.
     * @param usbFacingDirection  The direction the USB ports on the REV Hub are facing.
     * @return The current DrivetrainBuilder instance for method chaining.
     */
    public DrivetrainBuilder setRevHubOrientations(RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection, RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection) {
        this.logoFacingDirection = logoFacingDirection;
        this.usbFacingDirection = usbFacingDirection;
        return this;
    }

    public DrivetrainBuilder setStrafingMultiplier(double strafingMultiplier) {
        this.strafingMultiplier = strafingMultiplier;
        return this;
    }

    /**
     * Builds and returns a drivetrain based on the provided configuration.
     *
     * @return A {@link Drivetrain} instance of the specified type.
     * @throws IllegalStateException if any required component is not set or the type is not specified.
     */
    public Drivetrain build() {
        if (frontLeftMotor == null || backLeftMotor == null || frontRightMotor == null || backRightMotor == null) {
            throw new IllegalStateException("All motors must be set before building the drivetrain.");
        }
        if (imu == null && type == DrivetrainType.FIELD_CENTRIC_MECANUM) {
            throw new IllegalStateException("IMU must be set before building the drivetrain because " + "you are using a field-centric drivetrain.");
        }
        if (gamepad == null) {
            throw new IllegalStateException("Gamepad must be set before building the drivetrain.");
        }
        if (type == null) {
            throw new IllegalStateException("Drivetrain type must be specified.");
        }

        switch (type) {
            case FIELD_CENTRIC_MECANUM:
                FieldCentricMecanumDrivetrain fieldCentric = new FieldCentricMecanumDrivetrain(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad, imu, logoFacingDirection, usbFacingDirection, follower);

                fieldCentric.setStrafingMultiplier(strafingMultiplier);

                return fieldCentric;
            case ROBOT_CENTRIC_MECANUM:
                RobotCentricMecanumDrivetrain robotCentric = new RobotCentricMecanumDrivetrain(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad, follower);

                robotCentric.setStrafingMultiplier(strafingMultiplier);

                return robotCentric;
            case TANK_DRIVETRAIN:
                TankDrivetrain tankDrivetrain = new TankDrivetrain(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad, follower);

                tankDrivetrain.setStrafingMultiplier(strafingMultiplier);

                return tankDrivetrain;
            default:
                throw new IllegalStateException("Unknown drivetrain type.");
        }
    }

    /**
     * Enum representing the type of drivetrain to build.
     * - FIELD_CENTRIC_MECANUM: Uses an IMU for orientation-aware driving.
     * - ROBOT_CENTRIC_MECANUM: Drives based on robot's perspective without external orientation.
     * - TANK_DRIVETRAIN: For tank drivetrains. Allows turning and strafing.
     */
    public enum DrivetrainType {
        FIELD_CENTRIC_MECANUM, ROBOT_CENTRIC_MECANUM, TANK_DRIVETRAIN,
    }
}
