package org.firstinspires.ftc.teamcode.robot.lib;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class IntakeClawLib extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public static double CLAW_OPEN_POSITION = 0.0;
    public static double CLAW_CLOSED_POSITION = 0.0;
    public static double SWIVEL_FORWARD_POSITION = 0.0;
    public static double SWIVEL_BACKWARD_POSITION = 0.5;

    public enum ClawState {
        OPEN,
        CLOSED,
    }

    public enum SwivelState {
        FORWARD,
        BACKWARD,
    }

    private ClawState clawState = ClawState.OPEN;
    private SwivelState swivelState = SwivelState.FORWARD;


    public void init() {
        //setClawState(ClawState.OPEN);
        //setSwivelState(SwivelState.FORWARD);
    }

    public void setClawState(ClawState state) {
        clawState = state;
        switch (clawState) {
            case OPEN:
                moveClawToPosition(CLAW_OPEN_POSITION);
                break;

            case CLOSED:
                moveClawToPosition(CLAW_CLOSED_POSITION);
                break;
        }
    }

    public void setSwivelState(SwivelState state) {
        swivelState = state;
        switch (swivelState) {
            case FORWARD:
                moveSwivelToPosition(SWIVEL_FORWARD_POSITION);
                break;

            case BACKWARD:
                moveSwivelToPosition(SWIVEL_BACKWARD_POSITION);
                break;
        }
    }

    public void moveClawToPosition(double position) {
        robot.intakeClawServo.setPosition(position);
    }
    public void moveSwivelToPosition(double position) {
        robot.intakeSwivelServo.setPosition(position);
    }

    @Override
    public void periodic() {
        setClawState(clawState);
        setSwivelState(swivelState);
    }
}
