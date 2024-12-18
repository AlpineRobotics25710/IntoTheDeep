package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

public abstract class Arm implements Mechanism {
    protected Servo armServoLeft;
    protected Servo armServoRight;
    protected Servo wristServoLeft;
    protected Servo wristServoRight;

    protected MechanismState armState;

    public void setArmPosition(double position) {
        armServoLeft.setPosition(position);
        armServoRight.setPosition(position);
    }

    public void setWristPosition(double position) {
        wristServoLeft.setPosition(position);
        wristServoRight.setPosition(position);
    }
}
