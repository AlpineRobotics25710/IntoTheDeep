package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public class OuttakeClaw extends SubsystemBase {
    public static double CLAW_OPEN_POS = 1; // DONE
    public static double CLAW_CLOSED_POS = 0.65; // DONE
    public static double SWIVEL_SIDEWAYS_POS = 0.35; // DONE
    public static double SWIVEL_BOTTOM_POS = 0.7; // DONE
    public static double SWIVEL_TOP_POS = 0.03; // DONE

    private final Servo clawServo;
    private final Servo swivelServo;

    private OuttakeClawState clawState;
    private OuttakeSwivelState swivelState;

    public OuttakeClaw(Servo clawServo, Servo swivelServo) {
        this.clawServo = clawServo;
        this.swivelServo = swivelServo;
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

    public void setClawState(OuttakeClawState state) {
        clawState = state;
        switch (clawState) {
            case OPEN:
                setClawPosition(CLAW_OPEN_POS);
                break;

            case CLOSED:
                setClawPosition(CLAW_CLOSED_POS);
                break;
        }
    }

    public OuttakeSwivelState getSwivelState() {
        return swivelState;
    }

    public void setSwivelState(OuttakeSwivelState state) {
        swivelState = state;
        switch (swivelState) {
            case SIDEWAYS:
                setSwivelPosition(SWIVEL_SIDEWAYS_POS);
                break;

            case TOP:
                setSwivelPosition(SWIVEL_TOP_POS);
                break;

            case BOTTOM:
                setSwivelPosition(SWIVEL_BOTTOM_POS);
                break;
        }
    }

    public enum OuttakeSwivelState {
        SIDEWAYS, TOP, BOTTOM
    }

    public enum OuttakeClawState {
        OPEN, CLOSED
    }
}
