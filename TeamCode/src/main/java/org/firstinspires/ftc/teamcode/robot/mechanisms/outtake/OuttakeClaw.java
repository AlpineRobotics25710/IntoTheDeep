package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public class OuttakeClaw extends SubsystemBase {
    public static double CLAW_OPEN_POS = 0.3; // DONE
    public static double CLAW_CLOSED_POS = 0; // DONE
    public static double SWIVEL_WALL_INTAKE_POS = 0.45; // DONE
    public static double SWIVEL_TRANSFER_POS = 0.45; // DONE
    public static double SWIVEL_OUTTAKE_POS = 0.75; // DONE

    private final Servo clawServo;
    private final Servo swivelServo;

    private OuttakeClawState clawState;
    private OuttakeSwivelState swivelState;

    public OuttakeClaw(Servo clawServo, Servo swivelServo) {
        this.clawServo = clawServo;
        this.swivelServo = swivelServo;
        setClawState(OuttakeClawState.CLOSED);
        setSwivelState(OuttakeSwivelState.TRANSFER);
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
        TelemetryUtil.addData("Claw Servo Moving to", position);
    }

    public void setSwivelPosition(double position) {
        swivelServo.setPosition(position);
        TelemetryUtil.addData("Swivel Servo Moving to", position);
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

    public enum OuttakeSwivelState {
        WALL_INTAKE, TRANSFER, OUTTAKE
    }

    public enum OuttakeClawState {
        OPEN, CLOSED
    }
}
