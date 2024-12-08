package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

public class Arm implements Mechanism {
    private Servo wristServoLeft;
    private Servo wristServoRight;
    private Servo swivelServo;
    private HardwareMap hardwareMap;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR WRIST
    private final double WRIST_UP_POS = 0.0;
    private final double WRIST_DOWN_POS = 1.0;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR SWIVEL
    private final double SWIVEL_LEFT_POS = 0.0;
    private final double SWIVEL_RIGHT_POS = 1.0;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        wristServoLeft = hardwareMap.get(Servo.class, "wristServoLeft");
        wristServoRight = hardwareMap.get(Servo.class, "wristServoRight");
        swivelServo = hardwareMap.get(Servo.class, "swivelServo");

        wristServoLeft.setPosition(WRIST_DOWN_POS);
        wristServoRight.setPosition(WRIST_DOWN_POS);
        swivelServo.setPosition(SWIVEL_LEFT_POS);
    }

    @Override
    public void update() {
        Mechanism.super.update();
    }

    public void wristUp() {
        wristServoLeft.setPosition(WRIST_UP_POS);
        wristServoRight.setPosition(WRIST_UP_POS);
    }

    public void wristDown() {
        wristServoLeft.setPosition(WRIST_DOWN_POS);
        wristServoRight.setPosition(WRIST_DOWN_POS);
    }

    public void swivelLeft() {
        swivelServo.setPosition(SWIVEL_LEFT_POS);
    }

    public void swivelRight() {
        swivelServo.setPosition(SWIVEL_RIGHT_POS);
    }

    public double getWRIST_UP_POS() {
        return WRIST_UP_POS;
    }

    public double getSWIVEL_RIGHT_POS() {
        return SWIVEL_RIGHT_POS;
    }

    public double getWRIST_DOWN_POS() {
        return WRIST_DOWN_POS;
    }

    public double getSWIVEL_LEFT_POS() {
        return SWIVEL_LEFT_POS;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
}
