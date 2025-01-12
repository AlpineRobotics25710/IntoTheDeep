package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArm extends SubsystemBase {
    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR ARM
    public static double ARM_WALL_INTAKE_POS = 0; // COMPLETE
    public static double ARM_TRANSFER_POS = 0.5; // COMPLETE
    public static double ARM_OUTTAKE_FRONT_POS = 0.0;
    public static double ARM_OUTTAKE_BACK_POS = 0.0; // COMPLETE

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR WRIST
    public static double WRIST_WALL_INTAKE_POS = 1; // COMPLETE
    public static double WRIST_TRANSFER_POS = 0.0; // COMPLETE
    public static double WRIST_OUTTAKE_FRONT_POS = 0.0;
    public static double WRIST_OUTTAKE_BACK_POS = 0.5; // COMPLETE

    private final Servo armServoLeft;
    private final Servo armServoRight;
    private final Servo wristServo;

    private OuttakeArmState currentState;

    public OuttakeArm(Servo armServoRight, Servo armServoLeft, Servo wristServo) {
        this.armServoRight = armServoRight;
        this.armServoLeft = armServoLeft;
        this.wristServo = wristServo;
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

            case OUTTAKE_FRONT:
                setArmPosition(ARM_OUTTAKE_FRONT_POS);
                setWristPosition(WRIST_OUTTAKE_FRONT_POS);
                break;

            case OUTTAKE_BACK:
                setArmPosition(ARM_OUTTAKE_BACK_POS);
                setWristPosition(WRIST_OUTTAKE_BACK_POS);
                break;
        }
    }

    public void setWristPosition(double position) {
        wristServo.setPosition(position);
    }

    public double getWristPosition() {
        return wristServo.getPosition();
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
        WALL_INTAKE, TRANSFER, OUTTAKE_FRONT, OUTTAKE_BACK
    }
}
