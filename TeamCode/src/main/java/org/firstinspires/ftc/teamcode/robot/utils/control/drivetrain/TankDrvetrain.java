package org.firstinspires.ftc.teamcode.robot.utils.control.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.utils.control.gamepad.CustomGamepad;

public class TankDrvetrain extends Drivetrain {
    /**
     * Constructor for Drivetrain.
     * NOTE: Remember to initialize your wheel motors before calling this constructor. The motors are not
     * initialized by default.
     *
     * @param frontLeftMotor  the front left wheel motor
     * @param backLeftMotor   the back left wheel motor
     * @param frontRightMotor the front right wheel motor
     * @param backRightMotor  the back right wheel motor
     * @param gamepad         the gamepad to take input from
     */
    public TankDrvetrain(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, CustomGamepad gamepad) {
        super(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad);
    }

    @Override
    public void update() {
        double drive = gamepad.getLeftStick().getY();
        double turn = gamepad.getRightStick().getX();
        double strafe = gamepad.getLeftStick().getX();

        frontLeftMotor.setPower(Range.clip(drive + turn + strafe, -1, 1));
        frontRightMotor.setPower(Range.clip(drive - turn - strafe, -1, 1));
        backLeftMotor.setPower(Range.clip(drive + turn - strafe, -1, 1));
        backRightMotor.setPower(Range.clip(drive - turn + strafe, -1, 1));
    }
}
