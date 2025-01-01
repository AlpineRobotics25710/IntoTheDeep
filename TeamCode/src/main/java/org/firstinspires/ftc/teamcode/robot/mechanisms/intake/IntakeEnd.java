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
        if (currentState == ActiveState.HIGH) {
            setState(ActiveState.LOW);
        } else if (currentState == ActiveState.LOW){
            setState(ActiveState.OFF);
        } else {
            setState(ActiveState.HIGH);
        }
    }

    public void setState(ActiveState state) {
        currentState = state;
        switch (state) {
            case HIGH:
                intakeServo.setPower(1);
                break;

            case LOW:
                intakeServo.setPower(0.5);
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
        HIGH, LOW, OFF
    }
}
