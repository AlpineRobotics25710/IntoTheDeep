package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

public class Claw implements Mechanism {
    private Servo clawServo;
    private final HardwareMap hardwareMap;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE  POSITIONS FOR CLAW
    private final double OPEN_POS = 1.0;
    private final double CLOSED_POS = 0.0;

    public Claw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setPosition(CLOSED_POS);
    }

    public void openClaw() {
        clawServo.setPosition(OPEN_POS);
    }

    public void closeClaw() {
        clawServo.setPosition(CLOSED_POS);
    }

    @Override
    public void update() {
        Mechanism.super.update();
    }

    public double getOPEN_POS() {
        return OPEN_POS;
    }

    public double getCLOSED_POS() {
        return CLOSED_POS;
    }
}
