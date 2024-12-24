package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.lib.IntakeArmLib;
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

        setState(MechanismState.TRANSFER);
    }

    public void setState(MechanismState state) {
        currentState = state;
        switch (currentState) {
            case INTAKE:
                setArmPosition(ARM_INTAKE_POS);
                setWristPosition(WRIST_INTAKE_POS);
                break;

            case TRANSFER:
                setArmPosition(ARM_TRANSFER_POS);
                setWristPosition(WRIST_TRANSFER_POS);
                break;
        }
    }
}
