package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

@Config
public class IntakeArm implements Mechanism {
    private final HardwareMap hardwareMap;
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo wristServoLeft;
    private Servo wristServoRight;

    public static double ARM_INTAKE_POS = 0.0;
    public static double ARM_TRANSFER_POS = 0.0;
    public static double ARM_ASCENT_POS = 0.0;
    public static double WRIST_INTAKE_POS = 0.0;
    public static double WRIST_TRANSFER_POS = 0.0;
    public static double WRIST_ASCENT_POS = 0.0;

    public IntakeArmState intakeArmState;

    public enum IntakeArmState {
        ASCENT,
        INTAKE,
        TRANSFER,
    }

    public IntakeArm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        armServoLeft = hardwareMap.get(Servo.class, "iArmLeft");
        armServoRight = hardwareMap.get(Servo.class, "iArmRight");
        wristServoLeft = hardwareMap.get(Servo.class, "iWristLeft");
        wristServoRight = hardwareMap.get(Servo.class, "iWristRight");

        transfer();
    }

    public void setArmPosition(double position) {
        armServoLeft.setPosition(position);
        armServoRight.setPosition(position);
    }

    public void setWristPosition(double position) {
        wristServoLeft.setPosition(position);
        wristServoRight.setPosition(position);
    }

    public double getArmPosition() {
        return armServoRight.getPosition();
    }

    public void ascent() {
        armServoLeft.setPosition(ARM_ASCENT_POS);
        armServoRight.setPosition(ARM_ASCENT_POS);
        wristServoLeft.setPosition(WRIST_ASCENT_POS);
        wristServoRight.setPosition(WRIST_ASCENT_POS);
        intakeArmState = IntakeArmState.ASCENT;
    }

    public void intake() {
        armServoLeft.setPosition(ARM_INTAKE_POS);
        armServoRight.setPosition(ARM_INTAKE_POS);
        wristServoLeft.setPosition(WRIST_INTAKE_POS);
        wristServoRight.setPosition(WRIST_INTAKE_POS);
        intakeArmState = IntakeArmState.INTAKE;
    }

    public void transfer() {
        armServoLeft.setPosition(ARM_TRANSFER_POS);
        armServoRight.setPosition(ARM_TRANSFER_POS);
        wristServoLeft.setPosition(WRIST_TRANSFER_POS);
        wristServoRight.setPosition(WRIST_TRANSFER_POS);
        intakeArmState = IntakeArmState.TRANSFER;
    }
}
