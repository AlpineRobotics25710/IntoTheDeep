package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

public class IntakeArm implements Mechanism {
    private HardwareMap hardwareMap;
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo wristServo;

    public static final double ARM_UP_POS = 0.0;
    public static final double ARM_DOWN_POS = 1.0;

    public static final double WRIST_UP_POS = 0.0;
    public static final double WRIST_DOWN_POS = 1.0;

    public IntakeArm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        armServoLeft = hardwareMap.get(Servo.class, "leftArm");
        armServoRight = hardwareMap.get(Servo.class, "rightArm");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
    }

    public void setArmPosition(double position) {
        armServoLeft.setPosition(position);
        armServoRight.setPosition(position);
    }

    public void setWristPosition(double position) {
        wristServo.setPosition(position);
    }

    public void armUp() {
        armServoLeft.setPosition(ARM_UP_POS);
        armServoRight.setPosition(ARM_UP_POS);
    }

    public void armDown() {
        armServoLeft.setPosition(ARM_DOWN_POS);
        armServoRight.setPosition(ARM_DOWN_POS);
    }
}
