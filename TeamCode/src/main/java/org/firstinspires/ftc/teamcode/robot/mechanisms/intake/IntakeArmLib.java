package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.utils.RobotLib;
@Config
public class IntakeArmLib extends SubsystemBase {
    private final RobotLib robot = RobotLib.getInstance();
    public static double ARM_INTAKE_POS = 0.0;
    public static double ARM_TRANSFER_POS = 0.5;
    public static double WRIST_INTAKE_POS = 0.0;
    public static double WRIST_TRANSFER_POS = 0.5;
    public enum ArmState {
        INTAKE,
        TRANSFER,
    }

    public ArmState currentState;

    public IntakeArmLib() {
    }

    public void init() {
        //setState(ArmState.TRANSFER);
    }

    public void setState(ArmState state) {
        currentState = state;
        switch (currentState) {
            case INTAKE:
                moveArmToPosition(ARM_INTAKE_POS);
                moveWristToPosition(WRIST_INTAKE_POS);
                break;

            case TRANSFER:
                moveArmToPosition(ARM_TRANSFER_POS);
                moveWristToPosition(WRIST_TRANSFER_POS);
                break;
        }
    }

    public void moveArmToPosition(double position) {
        robot.iArmLeft.setPosition(position);
        robot.iArmRight.setPosition(position);
    }

    public void moveWristToPosition(double position) {
        robot.iWristLeft.setPosition(position);
        robot.iWristRight.setPosition(position);
    }

    @Override
    public void periodic() {
        setState(currentState);
    }
}
