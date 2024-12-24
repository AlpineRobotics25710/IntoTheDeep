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
    public static double ARM_LOW_CHAMBER_POS = 0.0;
    public static double ARM_HIGH_CHAMBER_POS = 0.0;
    public static double ARM_ASCENT_POS = 0.0;
    public static double ARM_OUTTAKE_POS = 0.0;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR WRIST
    public static double WRIST_WALL_INTAKE_POS = 1; // COMPLETE
    public static double WRIST_TRANSFER_POS = 0.0;
    public static double WRIST_LOW_CHAMBER_POS = 0.0;
    public static double WRIST_HIGH_CHAMBER_POS = 0.0;
    public static double WRIST_ASCENT_POS = 0.0;
    public static double WRIST_OUTTAKE_POS = 0.0;

    private final HardwareMap hardwareMap;

    public OuttakeArm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        armServoLeft = hardwareMap.get(Servo.class, "oArmLeft");
        armServoRight = hardwareMap.get(Servo.class, "oArmRight");
        wristServoLeft = hardwareMap.get(Servo.class, "leftEnd");
        wristServoRight = hardwareMap.get(Servo.class, "rightEnd");

        armServoRight.setDirection(Servo.Direction.REVERSE);
        wristServoLeft.setDirection(Servo.Direction.REVERSE);

        setState(MechanismState.TRANSFER);
    }

    @Override
    public void setState(MechanismState state) {
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

            case LOW_CHAMBER:
                setArmPosition(ARM_LOW_CHAMBER_POS);
                setWristPosition(WRIST_LOW_CHAMBER_POS);
                break;

            case HIGH_CHAMBER:
                setArmPosition(ARM_HIGH_CHAMBER_POS);
                setWristPosition(WRIST_HIGH_CHAMBER_POS);
                break;

            case ASCEND:
                setArmPosition(ARM_ASCENT_POS);
                setWristPosition(WRIST_ASCENT_POS);
                break;

            case LOW_BASKET:
            case HIGH_BASKET:
                setArmPosition(ARM_OUTTAKE_POS);
                setWristPosition(WRIST_OUTTAKE_POS);
                break;
        }
    }
}
