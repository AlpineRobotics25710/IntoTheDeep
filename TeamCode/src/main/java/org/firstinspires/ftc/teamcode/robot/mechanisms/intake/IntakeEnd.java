package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeEnd extends SubsystemBase {
    private final CRServo intakeServo;
    private ActiveState currentState;

    public IntakeEnd(CRServo intakeServo) {
        this.intakeServo = intakeServo;
    }

    public void toggleState() {
        if (currentState == ActiveState.FORWARD) {
            setState(ActiveState.REVERSE);
        } else if (currentState == ActiveState.REVERSE){
            setState(ActiveState.OFF);
        } else {
            setState(ActiveState.FORWARD);
        }
    }

    public void setState(ActiveState state) {
        currentState = state;
        switch (state) {
            case FORWARD:
                intakeServo.setPower(1);
                break;

            case REVERSE:
                intakeServo.setPower(-1);
                break;

            case OFF:
                intakeServo.setPower(0);
                break;
        }
    }

    public void setServoPower(double power) {
        intakeServo.setPower(power);
    }

    public ActiveState getCurrentState() {
        return currentState;
    }

    public enum ActiveState {
        FORWARD, REVERSE, OFF
    }
}
