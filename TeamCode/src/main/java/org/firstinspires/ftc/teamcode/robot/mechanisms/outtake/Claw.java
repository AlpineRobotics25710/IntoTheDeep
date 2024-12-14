package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

@Config
public class Claw implements Mechanism {
    private Servo clawServo;
    private Servo swivelServo;
    private final HardwareMap hardwareMap;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE  POSITIONS FOR CLAW
    private final double OPEN_POS = 1.0;
    private final double CLOSED_POS = 0.0;
    public final double SWIVEL_LEFT_POS = 0.0;
    public final double SWIVEL_STRAIGHT_POS = 0.5;
    public final double SWIVEL_RIGHT_POS = 1.0;

    public Claw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        swivelServo = hardwareMap.get(Servo.class, "swivelServo");

        clawServo.setPosition(CLOSED_POS);
        swivelServo.setPosition(SWIVEL_STRAIGHT_POS);
    }

    public void open() {
        clawServo.setPosition(OPEN_POS);
    }

    public void close() {
        clawServo.setPosition(CLOSED_POS);
    }

    public void setSwivelPosition(int position) {
        swivelServo.setPosition(position);
    }

    public double getOPEN_POS() {
        return OPEN_POS;
    }

    public double getCLOSED_POS() {
        return CLOSED_POS;
    }
}
