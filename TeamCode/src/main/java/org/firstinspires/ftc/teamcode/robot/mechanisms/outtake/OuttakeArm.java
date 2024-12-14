package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

@Config
public class OuttakeArm implements Mechanism {
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo wristServoLeft;
    private Servo wristServoRight;

    private final HardwareMap hardwareMap;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR WRIST
    public static double ARM_WALL_INTAKE_POS = 0.075; // COMPLETE
    public static double ARM_TRANSFER_POS = 0.95; // COMPLETE
    public static double ARM_LOW_CHAMBER_POS = 0.0;
    public static double ARM_HIGH_CHAMBER_POS = 0.0;
    public static double ARM_INIT_POS = ARM_TRANSFER_POS;


    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR SWIVEL
    public static double WRIST_WALL_INTAKE_POS = 1; // COMPLETE
    public static double WRIST_TRANSFER_POS = 0.0;
    public static double WRIST_LOW_CHAMBER_POS = 0.0;
    public static double WRIST_HIGH_CHAMBER_POS = 0.0;
    public static double WRIST_INIT_POS = WRIST_WALL_INTAKE_POS;

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

        armServoLeft.setPosition(ARM_INIT_POS);
        armServoRight.setPosition(ARM_INIT_POS);
        wristServoLeft.setPosition(WRIST_INIT_POS);
        wristServoRight.setPosition(WRIST_INIT_POS);
    }

    // FOR TESTNG PURPOSES ONLY
    public void setArmPosition(double position) {
        armServoLeft.setPosition(position);
        armServoRight.setPosition(position);
    }

    // FOR TESTNG PURPOSES ONLY
    public void setWristPosition(double position) {
        wristServoLeft.setPosition(position);
        wristServoRight.setPosition(position);
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
}
