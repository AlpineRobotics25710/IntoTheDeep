package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class Extendo extends SubsystemBase {
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    private static PIDFController extendoPIDF;
    private final DcMotor left; //we can make left our lead motor in this case
    private final DcMotor right;
    public boolean extendoReached;
    private double targetPosition = 0.0;
    private boolean manualMode;
    public final static double MAX_LENGTH = 0;

    public Extendo(DcMotor left, DcMotor right, boolean manualMode) {
        this.left = left;
        this.right = right;
        setTargetPosition(0);
        extendoPIDF.setTolerance(10);
        this.manualMode = manualMode;
        setManualMode(manualMode);
        extendoPIDF = new PIDFController(kP, kI, kD, kF);
    }

    //in this case the position is inputted in ticks of the motor, can be changed later
    public void setTargetPosition(double position) {
        targetPosition = position;
    }

    public void setSlidesPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    @Override
    public void periodic() {
        if (!manualMode) {
            double power = extendoPIDF.calculate(left.getCurrentPosition(), targetPosition);
            extendoReached = (targetPosition > 0 && extendoPIDF.atSetPoint()) || (left.getCurrentPosition() <= 5 && targetPosition == 0);
            setSlidesPower(power);
        }
    }

    public boolean isManualMode() {
        return manualMode;
    }

    public void setManualMode(boolean manualMode) {
        this.manualMode = manualMode;
        if (manualMode) {
            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
