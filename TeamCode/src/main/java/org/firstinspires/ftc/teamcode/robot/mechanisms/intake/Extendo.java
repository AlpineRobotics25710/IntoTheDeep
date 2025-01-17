package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public class Extendo extends SubsystemBase {
    // TODO: NEED TO FIND REAL VALUES
    public static final double MAX_LENGTH = 300;
    public static final double BASE_POS = 0;
    public static final double TRANSFER_POS = 50;
    public static double kP = 0.03;
    public static double kI = 0.0;
    public static double kD = 0.0001;
    private static PIDController extendoPID;
    private final DcMotor right;
    private double targetPosition = 0.0;
    private boolean manualMode;

    public Extendo(DcMotor right, boolean manualMode) {
        this.right = right;
        this.manualMode = manualMode;
        setManualMode(manualMode);
        extendoPID = new PIDController(kP, kI, kD);
        extendoPID.setTolerance(3);
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
            if (targetPosition == BASE_POS) {
                power -= 0.2;
            } else {
                power += 0.2;
            }

            setPower(power);
        }
        TelemetryUtil.addData("current position", right.getCurrentPosition());
        TelemetryUtil.addData("target position", targetPosition);
        TelemetryUtil.addData("Extendo Reached", extendoReached());
    }

    public boolean extendoReached() {
        return (extendoPID.atSetPoint() && targetPosition > 0) || (right.getCurrentPosition() <= 3 && targetPosition == BASE_POS);
    }

    public boolean isManualMode() {
        return manualMode;
    }

    public void setManualMode(boolean manualMode) {
        this.manualMode = manualMode;
        if (manualMode) {
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            //right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //testing something
        }
    }
}
