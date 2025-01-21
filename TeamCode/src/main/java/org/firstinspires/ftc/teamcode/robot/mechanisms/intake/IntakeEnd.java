package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

public class IntakeEnd extends SubsystemBase {
    private final CRServo intakeServo;
    private ActiveState currentState;

    public IntakeEnd(CRServo intakeServo) {
        this.intakeServo = intakeServo;
        setState(ActiveState.OFF);
    }

    public void toggleState() {
        if (currentState == ActiveState.FORWARD) {
            setState(ActiveState.REVERSED);
        } else if (currentState == ActiveState.REVERSED){
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

            case REVERSED:
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
        FORWARD, REVERSED, OFF
    }
    @Override
    public void periodic(){
        TelemetryUtil.addData("CASE", currentState);
    }
}
