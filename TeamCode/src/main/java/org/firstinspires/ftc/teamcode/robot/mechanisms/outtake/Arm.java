package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

@Config
public class Arm implements Mechanism {
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo wristServoLeft;
    private Servo wristServoRight;

    private HardwareMap hardwareMap;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR WRIST
    public final double ARM_UP_POS = 0.0;
    public final double ARM_DOWN_POS = 1.0;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR SWIVEL
    public final double WRIST_UP_POS = 0.0;
    public final double WRIST_DOWN_POS = 1.0;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        armServoLeft = hardwareMap.get(Servo.class, "leftArm");
        armServoRight = hardwareMap.get(Servo.class, "rightArm");
        //wristServoLeft = hardwareMap.get(Servo.class, "wristServoLeft");
        //wristServoRight = hardwareMap.get(Servo.class, "wristServoRight");

        armServoLeft.setPosition(ARM_DOWN_POS);
        armServoRight.setPosition(ARM_DOWN_POS);
        //wristServoLeft.setPosition(WRIST_DOWN_POS);
        //wristServoRight.setPosition(WRIST_DOWN_POS);
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

    public void armUp() {
        armServoLeft.setPosition(ARM_UP_POS);
        armServoRight.setPosition(ARM_UP_POS);
    }

    public void armDown() {
        armServoLeft.setPosition(ARM_DOWN_POS);
        armServoRight.setPosition(ARM_DOWN_POS);
    }

    public void wristUp() {
        wristServoLeft.setPosition(WRIST_UP_POS);
        wristServoRight.setPosition(WRIST_UP_POS);
    }

    public void wristDown() {
        wristServoLeft.setPosition(WRIST_DOWN_POS);
        wristServoRight.setPosition(WRIST_DOWN_POS);
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
}
