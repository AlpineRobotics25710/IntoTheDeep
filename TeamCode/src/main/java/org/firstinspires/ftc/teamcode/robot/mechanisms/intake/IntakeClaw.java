package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.MechanismState;

@Config
public class IntakeClaw extends Claw {
    private final HardwareMap hardwareMap;

    // TODO: NEED TO FIND RIGHT POSITIONS FOR INTAKE CLAW
    public static double CLAW_OPEN_POS = 0.3; // DONE
    public static double CLAW_CLOSED_POS = 0.55; // DONE
    public static double SWIVEL_INTAKE_POS = 0.0;
    public static double SWIVEL_TRANSFER_POS = 0.0;

    public IntakeClaw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void setState(MechanismState state) {
        clawState = state;
        switch (clawState) {
            case INTAKE:
                closeClaw();
                setSwivelPosition(SWIVEL_INTAKE_POS);
                break;

            case TRANSFER:
                openClaw();
                setSwivelPosition(SWIVEL_TRANSFER_POS);
                break;
        }
    }

    public void closeClaw() {
        setClawPosition(CLAW_CLOSED_POS);
    }

    public void openClaw() {
        setClawPosition(CLAW_OPEN_POS);
    }

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "intakeClaw");
        swivelServo = hardwareMap.get(Servo.class, "intakeSwivel");

        setState(MechanismState.TRANSFER);
    }
}
