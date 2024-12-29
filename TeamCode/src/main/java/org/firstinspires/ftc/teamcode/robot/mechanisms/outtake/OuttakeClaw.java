package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;

@Config
public class OuttakeClaw extends Claw {

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR CLAW
    public static double CLAW_OPEN_POS = 0.0;
    public static double CLAW_CLOSED_POS = 0.0;

    public static double SWIVEL_TRANSFER_POS = 0.0;
    public static double SWIVEL_INTAKE_POS = 0.0;
    public static double SWIVEL_OUTTAKE_POS = 0.0;

    public OuttakeClaw() {
        //clawServo = robot.outtakeClawServo;
      //  swivelServo = robot.outtakeSwivelServo;
    }

    @Override
    public void init() {
        setClawState(ClawState.CLOSED);
        setSwivelState(SwivelState.TRANSFER);
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
            case OUTTAKE:
                setSwivelPosition(SWIVEL_OUTTAKE_POS);
                break;
        }
    }
}
