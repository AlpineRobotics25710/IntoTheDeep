package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.lib.IntakeClawLib;
import org.firstinspires.ftc.teamcode.robot.lib.RobotLib;

public abstract class Claw extends SubsystemBase implements Mechanism {
    protected final RobotLib robot = RobotLib.getInstance();
    protected Servo clawServo;
    protected Servo swivelServo;

    public enum ClawState {
        OPEN, CLOSED,
    }

    public enum SwivelState {
        INTAKE, TRANSFER, OUTTAKE,
    }

    protected ClawState clawState;
    protected SwivelState swivelState;

    public abstract void setClawState(ClawState state);

    public abstract void setSwivelState(SwivelState state);

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
