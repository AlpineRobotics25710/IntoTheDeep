package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.MechanismState;

@Config
public class OuttakeArm extends Arm {
    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR ARM
    public static double ARM_WALL_INTAKE_POS = 0.075; // COMPLETE
    public static double ARM_TRANSFER_POS = 0.95; // COMPLETE
    public static double ARM_OUTTAKE_POS = 0.0;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR WRIST
    public static double WRIST_WALL_INTAKE_POS = 1; // COMPLETE
    public static double WRIST_TRANSFER_POS = 0.0;
    public static double WRIST_OUTTAKE_POS = 0.0;

    @Override
    public void init() {
        armServoRight.setDirection(Servo.Direction.REVERSE);
        wristServoLeft.setDirection(Servo.Direction.REVERSE);

        setState(ArmState.TRANSFER);
    }

    @Override
    public void setState(ArmState state) {
        currentState = state;
        switch (currentState) {
            case INTAKE:
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
}
