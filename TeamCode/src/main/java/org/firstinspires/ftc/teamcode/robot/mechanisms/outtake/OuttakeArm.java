package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArm extends SubsystemBase {
    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR ARM
    public static double ARM_WALL_INTAKE_POS = 0; // COMPLETE
    public static double ARM_TRANSFER_POS = 0.8; // COMPLETE
    public static double ARM_SAMPLE_FRONT_POS = 0.0;
    public static double ARM_SAMPLE_BACK_POS = 0.0;
    public static double ARM_SPECIMEN_FRONT_POS = 0.0;
    public static double ARM_SPECIMEN_BACK_POS = 0.0;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR WRIST
    public static double WRIST_WALL_INTAKE_POS = 1; // COMPLETE
    public static double WRIST_TRANSFER_POS = 0.0;
    public static double WRIST_SAMPLE_FRONT_POS = 0.0;
    public static double WRIST_SAMPLE_BACK_POS = 0.0;
    public static double WRIST_SPECIMEN_FRONT_POS = 0.0;
    public static double WRIST_SPECIMEN_BACK_POS = 0.0;

    private final Servo armServoLeft;
    private final Servo armServoRight;
    private final Servo wristServoLeft;
    private final Servo wristServoRight;

    private OuttakeArmState currentState;

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

            case SAMPLE_FRONT:
                setArmPosition(ARM_SAMPLE_FRONT_POS);
                setWristPosition(WRIST_SAMPLE_FRONT_POS);
                break;

            case SAMPLE_BACK:
                setArmPosition(ARM_SAMPLE_BACK_POS);
                setWristPosition(WRIST_SAMPLE_BACK_POS);
                break;

            case SPECIMEN_FRONT:
                setArmPosition(ARM_SPECIMEN_FRONT_POS);
                setWristPosition(WRIST_SPECIMEN_FRONT_POS);
                break;

            case SPECIMEN_BACK:
                setArmPosition(ARM_SPECIMEN_BACK_POS);
                setWristPosition(WRIST_SPECIMEN_BACK_POS);
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
        WALL_INTAKE, TRANSFER, SAMPLE_FRONT, SAMPLE_BACK, SPECIMEN_FRONT, SPECIMEN_BACK
    }
}
