package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Arm;

@Config
public class IntakeArm extends Arm {
    public static double ARM_INTAKE_POS = 0.0;
    public static double ARM_TRANSFER_POS = 0.0;
    public static double WRIST_INTAKE_POS = 0.0;
    public static double WRIST_TRANSFER_POS = 0.0;

    public IntakeArm() {
        armServoRight = robot.iArmRight;
        armServoLeft = robot.iArmLeft;
        wristServoRight = robot.iWristRight;
        wristServoLeft = robot.iWristLeft;
    }

    @Override
    public void init() {
        armServoRight.setDirection(Servo.Direction.REVERSE);
        armServoLeft.setDirection(Servo.Direction.FORWARD);

        wristServoRight.setDirection(Servo.Direction.FORWARD);
        wristServoLeft.setDirection(Servo.Direction.REVERSE);

        setState(ArmState.TRANSFER);
    }

    public void setState(ArmState state) {
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
