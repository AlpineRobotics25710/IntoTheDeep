package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class Claw extends SubsystemBase implements Mechanism {
    public enum ClawState {
        OPEN,
        CLOSED,
    }

    public enum SwivelState {
        INTAKE,
        TRANSFER,
    }
    protected Servo clawServo;
    protected Servo swivelServo;
    protected ClawState clawState;
    protected SwivelState swivelState;

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }

    public void setSwivelPosition(double position) {
        swivelServo.setPosition(position);
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }

    public double getSwivelPosition() {
        return swivelServo.getPosition();
    }
}
