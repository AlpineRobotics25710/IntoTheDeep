package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.MechanismState;

@Config
public class OuttakeArmLib extends Arm {
    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR WRIST
    public static double ARM_WALL_INTAKE_POS = 0.075; // COMPLETE
    public static double ARM_TRANSFER_POS = 0.95; // COMPLETE
    public static double ARM_LOW_CHAMBER_POS = 0.0;
    public static double ARM_HIGH_CHAMBER_POS = 0.0;
    public static double ARM_ASCENT_POS = 0.0;
    public static double ARM_INIT_POS = ARM_TRANSFER_POS;
    public static double WRIST_WALL_INTAKE_POS = 1; // COMPLETE
    public static double WRIST_TRANSFER_POS = 0.0;
    public static double WRIST_LOW_CHAMBER_POS = 0.0;
    public static double WRIST_HIGH_CHAMBER_POS = 0.0;
    public static double WRIST_ASCENT_POS = 0.0;

    public static double ARM_WALL_POS = 0.0;
    public static double WRIST_WALL_POS = 0.0;
    private final HardwareMap hardwareMap;

    public OuttakeArmLib(HardwareMap hardwareMap) {
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

        transfer();
    }

    public void ascend() {
//        setArmPosition(ARM_ASCENT_POS);
//        setWristPosition(WRIST_ASCENT_POS);
        armState = MechanismState.IDLE;
    }

    public void intake() {
 //       setArmPosition(ARM_WALL_INTAKE_POS);
   //     setWristPosition(WRIST_WALL_INTAKE_POS);
        armState = MechanismState.IDLE;
    }

    public void transfer() {
        setArmPosition(ARM_TRANSFER_POS);
        setWristPosition(WRIST_TRANSFER_POS);
        armState = MechanismState.TRANSFER;
    }

    public void lowChamber() {
        setArmPosition(ARM_LOW_CHAMBER_POS);
        setWristPosition(WRIST_LOW_CHAMBER_POS);
        armState = MechanismState.LOW_CHAMBER;
    }

    public void highChamber() {
        setArmPosition(ARM_HIGH_CHAMBER_POS);
        setWristPosition(WRIST_HIGH_CHAMBER_POS);
        armState = MechanismState.HIGH_CHAMBER;
    }

    @Override
    public void lowBasket() {
        setArmPosition(ARM_LOW_CHAMBER_POS);
        setWristPosition(WRIST_LOW_CHAMBER_POS);
        armState = MechanismState.LOW_BASKET;
    }

    @Override
    public void highBasket() {
        setArmPosition(ARM_HIGH_CHAMBER_POS);
        setWristPosition(WRIST_HIGH_CHAMBER_POS);
        armState = MechanismState.HIGH_BASKET;
    }

    public void grabOffWall() {
        setArmPosition(ARM_WALL_POS);
        setArmPosition(WRIST_WALL_POS);
        armState = MechanismState.WALL_GRAB;
    }
}
