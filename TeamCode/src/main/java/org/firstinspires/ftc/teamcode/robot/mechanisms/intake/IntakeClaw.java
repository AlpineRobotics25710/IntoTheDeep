package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;

@Config
public class IntakeClaw extends Claw {
    // TODO: NEED TO FIND RIGHT POSITIONS FOR INTAKE CLAW
    public static double CLAW_OPEN_POS = 0.3; // DONE
    public static double CLAW_CLOSED_POS = 0.55; // DONE
    public static double SWIVEL_INTAKE_POS = 0.0;
    public static double SWIVEL_TRANSFER_POS = 0.0;

    public IntakeClaw() {
        clawServo = robot.intakeClaw;
        swivelServo = robot.intakeSwivel;
    }

    @Override
    public void init() {
        setClawState(ClawState.CLOSED);
        setSwivelState(SwivelState.INTAKE);
    }

    public void setClawState(ClawState state) {
        clawState = state;
        switch (clawState) {
            case OPEN:
                setClawPosition(CLAW_OPEN_POS);
                break;

            case CLOSED:
                setSwivelPosition(CLAW_CLOSED_POS);
                break;
        }
    }

    public void setSwivelState(SwivelState state) {
        swivelState = state;
        switch (swivelState) {
            case INTAKE:
                setSwivelPosition(SWIVEL_INTAKE_POS);
                break;

            case TRANSFER:
                setSwivelPosition(SWIVEL_TRANSFER_POS);
                break;
        }
    }
}
