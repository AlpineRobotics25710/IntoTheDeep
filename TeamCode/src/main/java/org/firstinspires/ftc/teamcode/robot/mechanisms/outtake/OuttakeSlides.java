package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

// Bro i literally just copy pasted the extendo class and changed the name of the constructor if this works imma make a super class and test that
@Config
public class OuttakeSlides extends SubsystemBase {
    // TODO: NEED TO FIND REAL VALUES
    public static final double TRANSFER_POS = 0.0;
    public static final double HIGH_BASKET = 0.0;
    public static final double LOW_BASKET = 0.0;
    public static final double LOW_CHAMBER = 0.0;
    public static final double HIGH_CHAMBER = 0.0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    private static PIDFController outtakePIDF;
    private final DcMotor left; //we can make left our lead motor in this case
    private final DcMotor right;
    public boolean extendoReached;
    private double targetPosition = 0.0;
    private boolean manualMode;

    public OuttakeSlides(DcMotor left, DcMotor right, boolean manualMode) {
        this.left = left;
        this.right = right;
        setTargetPosition(0);
        outtakePIDF.setTolerance(10);
        this.manualMode = manualMode;
        setManualMode(manualMode);
        outtakePIDF = new PIDFController(kP, kI, kD, kF);
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
            double power = outtakePIDF.calculate(left.getCurrentPosition(), targetPosition);
            extendoReached = (targetPosition > 0 && outtakePIDF.atSetPoint()) || (left.getCurrentPosition() <= 5 && targetPosition == 0);
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
