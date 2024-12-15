package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;

@Config
public class IntakeClaw extends Claw {
    private final HardwareMap hardwareMap;

    // TODO: NEED TO FIND RIGHT POSITIONS FOR INTAKE CLAW
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
}
