package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.lib.IntakeArmLib;
import org.firstinspires.ftc.teamcode.robot.lib.RobotLib;

public abstract class Arm extends SubsystemBase implements Mechanism {
    private final RobotLib robot = RobotLib.getInstance();
    protected Servo armServoLeft = robot.iArmLeft;
    protected Servo armServoRight = robot.iArmRight;
    protected Servo wristServoLeft = robot.iWristLeft;
    protected Servo wristServoRight = robot.iWristRight;

    public enum ArmState {
        INTAKE,
        TRANSFER,
        OUTTAKE,
    }

    public ArmState currentState;

    public abstract void setState(ArmState state);

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
