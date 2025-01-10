package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class Extendo extends SubsystemBase {
    // TODO: NEED TO FIND REAL VALUES
    public static final double MAX_LENGTH = 0.0;
    public static final double BASE_POS = 0.0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    private static PIDFController extendoPIDF;
    private final DcMotor right;
    public boolean extendoReached;
    private double targetPosition = 0.0;
    private boolean manualMode;

    public Extendo(DcMotor right, boolean manualMode) {
        this.right = right;
       // setTargetPosition(0);
        //extendoPIDF.setTolerance(10);
        this.manualMode = manualMode;
        setManualMode(manualMode);
        extendoPIDF = new PIDFController(kP, kI, kD, kF);
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
            double power = extendoPIDF.calculate(right.getCurrentPosition(), targetPosition);
            extendoReached = (targetPosition > 0 && extendoPIDF.atSetPoint()) || (right.getCurrentPosition() <= 5 && targetPosition == 0);
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
