package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public class IntakeArm extends SubsystemBase {
    public static double ARM_INTAKE_POS = 0.56;
    public static double ARM_TRANSFER_POS = 0.15;
    public static double WRIST_INTAKE_POS = 0.23;
    public static double WRIST_TRANSFER_POS = 0.0;
    private IntakeArmState currentState;
    private final Servo armServoLeft;
    private final Servo armServoRight;
    private final Servo wristServoRight;

    public IntakeArm(Servo armServoRight, Servo armServoLeft, Servo wristServoRight) {
        this.armServoRight = armServoRight;
        this.armServoLeft = armServoLeft;
        this.wristServoRight = wristServoRight;

        setState(IntakeArmState.TRANSFER);
    }

    public void setState(IntakeArmState state) {
        currentState = state;
        switch (currentState) {
            case INTAKE:
                setArmPosition(ARM_INTAKE_POS);
                setWristPosition(WRIST_INTAKE_POS);
                break;

            case TRANSFER:
                setArmPosition(ARM_TRANSFER_POS);
                setWristPosition(WRIST_TRANSFER_POS);
                break;
        }
        TelemetryUtil.addData("Current Arm State", currentState);
    }

    public void setWristPosition(double position) {
        wristServoRight.setPosition(position);
    }

    public double getWristPosition() {
        return wristServoRight.getPosition();
    }

    public void setArmPosition(double position) {
        armServoLeft.setPosition(position);
        armServoRight.setPosition(position);
    }

    public double getArmPosition() {
        return armServoRight.getPosition();
    }

    public IntakeArmState getCurrentState() {
        return currentState;
    }

    public enum IntakeArmState {
        INTAKE, TRANSFER,
    }
}
