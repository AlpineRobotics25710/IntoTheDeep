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
    public static double OPEN_POS = 0.0;
    public static double CLOSED_POS = 0.0;

    public static double SWIVEL_INTAKE_POS = 0.0;
    public static double SWIVEL_OUTTAKE_POS = 0.0;
    public static double SWIVEL_INIT_POS = SWIVEL_INTAKE_POS;

    public Claw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "outtakeClaw");
        swivelServo = hardwareMap.get(Servo.class, "outtakeClawSwivel");

        clawServo.setPosition(CLOSED_POS);
        swivelServo.setPosition(SWIVEL_INIT_POS);
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }

    public void setSwivelPosition(double position) {
        swivelServo.setPosition(position);
    }
}
