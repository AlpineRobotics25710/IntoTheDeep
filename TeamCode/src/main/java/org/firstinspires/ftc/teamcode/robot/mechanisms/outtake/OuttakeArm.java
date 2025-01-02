package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArm extends SubsystemBase {
    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR ARM
    public static double ARM_WALL_INTAKE_POS = 0; // COMPLETE
    public static double ARM_TRANSFER_POS = 0.8; // COMPLETE
    public static double ARM_OUTTAKE_POS = 0.0;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR WRIST
    public static double WRIST_WALL_INTAKE_POS = 1; // COMPLETE
    public static double WRIST_TRANSFER_POS = 0.0;
    public static double WRIST_OUTTAKE_POS = 0.0;

    private final Servo armServoLeft;
    private final Servo armServoRight;
    private final Servo wristServoLeft;
    private final Servo wristServoRight;

    public static OuttakeArmState currentState;

    public OuttakeArm(Servo armServoRight, Servo armServoLeft, Servo wristServoRight, Servo wristServoLeft) {
        this.armServoRight = armServoRight;
        this.armServoLeft = armServoLeft;
        this.wristServoRight = wristServoRight;
        this.wristServoLeft = wristServoLeft;
    }

    public void init() {
        armServoRight.setDirection(Servo.Direction.REVERSE);
        wristServoLeft.setDirection(Servo.Direction.REVERSE);

        setState(OuttakeArmState.TRANSFER);
    }

    public void setState(OuttakeArmState state) {
        currentState = state;
        switch (currentState) {
            case WALL_INTAKE:
                setArmPosition(ARM_WALL_INTAKE_POS);
                setWristPosition(WRIST_WALL_INTAKE_POS);
                break;

            case TRANSFER:
                setArmPosition(ARM_TRANSFER_POS);
                setWristPosition(WRIST_TRANSFER_POS);
                break;
            case OUTTAKE:
                setArmPosition(ARM_OUTTAKE_POS);
                setWristPosition(WRIST_OUTTAKE_POS);
                break;
        }
    }

    public void setWristPosition(double position) {
        wristServoLeft.setPosition(position);
        wristServoRight.setPosition(position);
    }

    public double getWristPosition() {
        return wristServoLeft.getPosition();
    }

    public void setArmPosition(double position) {
        armServoLeft.setPosition(position);
        armServoRight.setPosition(position);
    }

    public double getArmPosition() {
        return armServoRight.getPosition();
    }

    public OuttakeArmState getCurrentState() {
        return currentState;
    }

    public enum OuttakeArmState {
        WALL_INTAKE, TRANSFER, OUTTAKE,
    }
}
