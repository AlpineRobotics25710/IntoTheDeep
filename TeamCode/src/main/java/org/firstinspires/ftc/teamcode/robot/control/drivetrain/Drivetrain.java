package org.firstinspires.ftc.teamcode.robot.control.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;

public abstract class Drivetrain {
    protected DcMotor frontLeftMotor;
    protected DcMotor backLeftMotor;
    protected DcMotor frontRightMotor;
    protected DcMotor backRightMotor;
    protected CustomGamepad gamepad;

    /**
     * Multiplier to counteract imperfect strafing. Multiplied by the x-value of the left joystick
     */
    protected double strafingMultiplier = 1.1;

    /**
     * Constructor for Drivetrain.
     * NOTE: Remember to initialize your wheel motors before calling this constructor. The motors are not
     * initialized by default. For example, you must set the zero power behavior of the motors yourself.
     * Also remember to reverse the right motors.
     *
     * @param frontLeftMotor  the front left wheel motor
     * @param backLeftMotor   the back left wheel motor
     * @param frontRightMotor the front right wheel motor
     * @param backRightMotor  the back right wheel motor
     * @param gamepad         the gamepad to take input from
     */
    public Drivetrain(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, CustomGamepad gamepad) {
        this.frontLeftMotor = frontLeftMotor;
        this.backLeftMotor = backLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backRightMotor = backRightMotor;
        this.gamepad = gamepad;
    }

    /**
     * Sets the strafing multiplier to counteract imperfect strafing
     *
     * @param strafingMultiplier the strafing multiplier
     */
    public void setStrafingMultiplier(double strafingMultiplier) {
        this.strafingMultiplier = strafingMultiplier;
    }

    /**
     * Updates the program and moves the robot based on gamepad inputs. Should be called continuously
     * in the opmode loop. Pressing options (PS) or start (Xbox) will reset the yaw value of the IMU
     * when running a field centric mecanum drive train.
     */
    public abstract void update();
}
