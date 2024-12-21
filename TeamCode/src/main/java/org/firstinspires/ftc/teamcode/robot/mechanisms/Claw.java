package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

public abstract class Claw implements Mechanism {
    protected Servo clawServo;
    protected Servo swivelServo;

    protected MechanismState clawState;

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
