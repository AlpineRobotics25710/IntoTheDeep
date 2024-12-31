package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeClaw extends SubsystemBase {
    public static double CLAW_OPEN_POS = 0.1; // DONE
    public static double CLAW_CLOSED_POS = 0.3; // DONE
    public static double SWIVEL_WALL_INTAKE_POS = 0.0;
    public static double SWIVEL_TRANSFER_POS = 0.3; // DONE
    public static double SWIVEL_OUTTAKE_POS = 0.65; // DONE

    private final Servo clawServo;
    private final Servo swivelServo;

    private OuttakeClawState clawState;
    private OuttakeSwivelState swivelState;

    public OuttakeClaw(Servo clawServo, Servo swivelServo) {
        this.clawServo = clawServo;
        this.swivelServo = swivelServo;
    }

    public void init() {
        setClawState(OuttakeClawState.CLOSED);
        setSwivelState(OuttakeSwivelState.TRANSFER);
    }

    public void setClawState(OuttakeClawState state) {
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

    public void setSwivelState(OuttakeSwivelState state) {
        swivelState = state;
        switch (swivelState) {
            case WALL_INTAKE:
                setSwivelPosition(SWIVEL_WALL_INTAKE_POS);
                break;

            case TRANSFER:
                setSwivelPosition(SWIVEL_TRANSFER_POS);
                break;

            case OUTTAKE:
                setSwivelPosition(SWIVEL_OUTTAKE_POS);
                break;
        }
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }

    public void setSwivelPosition(double position) {
        swivelServo.setPosition(position);
    }

    public OuttakeClawState getClawState() {
        return clawState;
    }

    public OuttakeSwivelState getSwivelState() {
        return swivelState;
    }

    public enum OuttakeSwivelState {
        WALL_INTAKE, TRANSFER, OUTTAKE
    }

    public enum OuttakeClawState {
        OPEN, CLOSED
    }
}
