package org.firstinspires.ftc.teamcode.robot.utils.gamepad.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class Drivetrain {
    protected DcMotor frontLeftMotor;
    protected DcMotor backLeftMotor;
    protected DcMotor frontRightMotor;
    protected DcMotor backRightMotor;
    protected Gamepad gamepad;

    /**
     * Multiplier to counteract imperfect strafing. Multiplied by the x-value of the left joystick
     */
    protected double strafingMultiplier = 1.1;

    /**
     * Represents how sensitive the joysticks should be
     */
    protected double sensitivity;

    /**
     * Constructor for FieldCentricMecanumDrivetrain.
     * NOTE: Remember to initialize your wheel motors before calling this constructor. The motors are not
     * initialized by default.
     * @param frontLeftMotor the front left wheel motor
     * @param backLeftMotor the back left wheel motor
     * @param frontRightMotor the front right wheel motor
     * @param backRightMotor the back right wheel motor
     * @param gamepad the gamepad to take input from
     */
    public Drivetrain(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, Gamepad gamepad) {
        this.frontLeftMotor = frontLeftMotor;
        this.backLeftMotor = backLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backRightMotor = backRightMotor;
        this.gamepad = gamepad;
    }

    /**
     * Sets the strafing multiplier to counteract imperfect strafing
     * @param strafingMultiplier the strafing multiplier
     */
    public void setStrafingMultiplier(double strafingMultiplier) {
        this.strafingMultiplier = strafingMultiplier;
    }

    /**
     * Sets teh sensitivity of the joysticks
     */
    public void setSensitivity(double sensitivity) {
        this.sensitivity = sensitivity;
    }

    /**
     * Returns the strafing multiplier.
     * @return the strafing multiplier
     */
    public double getStrafingMultiplier() {
        return strafingMultiplier;
    }

    /**
     * Returns the controller sensitivity.
     * @return the controller sensitivity.
     */
    public double getSensitivity() {
        return sensitivity;
    }

    /**
     * Runs initialization code for the drivetrain. Should be called during robot initialization, before waitForStart().
     * DOES NOT INITIALIZE THE WHEEL MOTORS.
     */
    public abstract void init();

    /**
     * Updates the program and moves the robot based on gamepad inputs. Should be called continuously
     * in the opmode loop. Pressing options (PS) or start (Xbox) will reset the yaw value of the IMU.
     */
    public abstract void update();
}
