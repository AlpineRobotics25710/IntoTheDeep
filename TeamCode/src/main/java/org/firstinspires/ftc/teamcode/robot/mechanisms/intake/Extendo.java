package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class Extendo extends SubsystemBase {
    // TODO: NEED TO FIND REAL VALUES
    public static final double MAX_LENGTH = 650.0;
    public static final double BASE_POS = 100.0;
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
    private static PIDController extendoPID;
    private final DcMotor right;
    private double targetPosition = 0.0;
    private boolean manualMode;

    public Extendo(DcMotor right, boolean manualMode) {
        this.right = right;
        this.manualMode = manualMode;
        setManualMode(manualMode);
        extendoPID = new PIDController(kP, kI, kD);
        extendoPID.setTolerance(10);
        setTargetPosition(BASE_POS);
    }

    //in this case the position is inputted in ticks of the motor, can be changed later
    public void setTargetPosition(double position) {
        targetPosition = position;
    }

    public void setPower(double power) {
        right.setPower(power);
    }

    @Override
    public void periodic() {
        if (!manualMode) {
            extendoPID.setPID(kP, kI, kD);
            double power = extendoPID.calculate(right.getCurrentPosition(), targetPosition);
            setPower(power);
        }
    }

    public boolean isManualMode() {
        return manualMode;
    }

    public void setManualMode(boolean manualMode) {
        this.manualMode = manualMode;
        if (manualMode) {
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
