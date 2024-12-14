package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

@Config
public class IntakeClaw implements Mechanism {
    private final HardwareMap hardwareMap;
    private Servo clawServo;
    private Servo swivelServo;

    public static double CLAW_OPEN_POS = 0.0;
    public static double CLAW_CLOSE_POS = 0.0;
    public static double SWIVEL_INTAKE_POS = 0.0;
    public static double SWIVEL_TRANSFER_POS = 0.0;
    public static double SWIVEL_ASCENT_POS = 0.0;

    public IntakeClaw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "intakeClaw");
        swivelServo = hardwareMap.get(Servo.class, "intakeSwivel");
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }

    public void setSwivelPosition(double position) {
        swivelServo.setPosition(position);
    }
}
