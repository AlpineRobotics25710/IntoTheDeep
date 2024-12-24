package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class Arm extends SubsystemBase implements Mechanism {
    protected Servo armServoLeft;
    protected Servo armServoRight;
    protected Servo wristServoLeft;
    protected Servo wristServoRight;
    protected MechanismState currentState;

    public abstract void setState(MechanismState state);

    public void setArmPosition(double position) {
        armServoLeft.setPosition(position);
        armServoRight.setPosition(position);
    }

    public void setWristPosition(double position) {
        wristServoLeft.setPosition(position);
        wristServoRight.setPosition(position);
    }

    public double getArmPosition() {
        return armServoRight.getPosition();
    }

    public double getSwivelPosition() {
        return armServoLeft.getPosition();
    }
}
