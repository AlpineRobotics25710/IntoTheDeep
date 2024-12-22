package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.MechanismState;

@Config
public class IntakeArm extends Arm {
    private final HardwareMap hardwareMap;

    public static double ARM_INTAKE_POS = 0.0;
    public static double ARM_TRANSFER_POS = 0.0;
    public static double ARM_ASCENT_POS = 0.0;
    public static double WRIST_INTAKE_POS = 0.0;
    public static double WRIST_TRANSFER_POS = 0.0;
    public static double WRIST_ASCENT_POS = 0.0;

    public IntakeArm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        armServoLeft = hardwareMap.get(Servo.class, "iArmLeft");
        armServoRight = hardwareMap.get(Servo.class, "iArmRight");
        wristServoLeft = hardwareMap.get(Servo.class, "iWristLeft");
        wristServoRight = hardwareMap.get(Servo.class, "iWristRight");

        armServoRight.setDirection(Servo.Direction.REVERSE);
        armServoLeft.setDirection(Servo.Direction.FORWARD);

        wristServoRight.setDirection(Servo.Direction.FORWARD);
        wristServoLeft.setDirection(Servo.Direction.REVERSE);

        transfer();
    }

    @Override
    public void ascend() {
        setArmPosition(ARM_ASCENT_POS);
        setWristPosition(WRIST_ASCENT_POS);
        armState = MechanismState.ASCENT;
    }

    @Override
    public void intake() {
        setArmPosition(ARM_INTAKE_POS);
        setWristPosition(WRIST_INTAKE_POS);
        armState = MechanismState.INTAKE;
    }

    @Override
    public void lowChamber() {
        intake();
        armState = MechanismState.LOW_CHAMBER;
    }

    @Override
    public void highChamber() {
        intake();
        armState = MechanismState.HIGH_CHAMBER;
    }

    @Override
    public void lowBasket() {
        intake();
        armState = MechanismState.LOW_BASKET;
    }

    @Override
    public void highBasket() {
        intake();
        armState = MechanismState.HIGH_BASKET;
    }

    @Override
    public void transfer() {
        setArmPosition(ARM_TRANSFER_POS);
        setWristPosition(WRIST_TRANSFER_POS);
        armState = MechanismState.TRANSFER;
    }
}
