package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OuttakeArm extends SubsystemBase {
    // Arm positions
    public static double ARM_WALL_INTAKE_FRONT_POS = 0.65; // Adjust as needed
    public static double ARM_WALL_INTAKE_BACK_POS = 0.2; // Adjust as needed
    public static double ARM_TRANSFER_POS = 0.55; // COMPLETE
    public static double ARM_OUTTAKE_FRONT_POS = 0.5;
    public static double ARM_OUTTAKE_BACK_POS = 0.1; // COMPLETE
    public static double ARM_INTERMEDIATE_POS = 0.4;
    public static double ARM_DEPOSIT_SAMPLE_POS = 0.05; // Adjust as needed
    public static double ARM_INIT_POS = 0.57;

    // Wrist positions
    public static double WRIST_WALL_INTAKE_FRONT_POS = 0.65; // Adjust as needed
    public static double WRIST_WALL_INTAKE_BACK_POS = 0.8; // Adjust as needed
    public static double WRIST_TRANSFER_POS = 0.1; // COMPLETE
    public static double WRIST_OUTTAKE_FRONT_POS = 1;
    public static double WRIST_OUTTAKE_BACK_POS = 0.65; // COMPLETE
    public static double WRIST_INTERIM_POS = 0.65;
    public static double WRIST_DEPOSIT_SAMPLE_POS = 0.4; // Adjust as needed
    public static double WRIST_INIT_POS = 1.0;

    private final Servo armServoLeft;
    private final Servo armServoRight;
    private final Servo wristServo;

    private OuttakeArmState currentState;

    public OuttakeArm(Servo armServoRight, Servo armServoLeft, Servo wristServo) {
        this.armServoRight = armServoRight;
        this.armServoLeft = armServoLeft;
        this.wristServo = wristServo;
    }

    public void setState(OuttakeArmState state) {
        currentState = state;
        switch (currentState) {
            case WALL_INTAKE_FRONT:
                setArmPosition(ARM_WALL_INTAKE_FRONT_POS);
                setWristPosition(WRIST_WALL_INTAKE_FRONT_POS);
                break;

            case WALL_INTAKE_BACK:
                setArmPosition(ARM_WALL_INTAKE_BACK_POS);
                setWristPosition(WRIST_WALL_INTAKE_BACK_POS);
                break;

            case INTERMEDIATE:
                setArmPosition(ARM_INTERMEDIATE_POS);
                setWristPosition(WRIST_INTERIM_POS);

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

            case DEPOSIT_SAMPLE:
                setArmPosition(ARM_DEPOSIT_SAMPLE_POS);
                setWristPosition(WRIST_DEPOSIT_SAMPLE_POS);
                break;
            case INIT:
                setArmPosition(ARM_INIT_POS);
                setWristPosition(WRIST_INIT_POS);
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
        WALL_INTAKE_FRONT,
        WALL_INTAKE_BACK,
        TRANSFER,
        INTERMEDIATE,
        OUTTAKE_FRONT,
        OUTTAKE_BACK,
        DEPOSIT_SAMPLE,
        INIT,
    }
}
