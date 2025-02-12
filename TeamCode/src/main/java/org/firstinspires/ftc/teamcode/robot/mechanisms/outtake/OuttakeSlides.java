package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

// Bro i literally just copy pasted the extendo class and changed the name of the constructor if this works imma make a super class and test that
@Config
public class OuttakeSlides extends SubsystemBase {
    public static final double TRANSFER_POS = 0.0;
    public static final double HIGH_BASKET = 1200;
    public static final double LOW_BASKET = 150.0;
    public static final double LOW_CHAMBER = 0.0;
    public static final double HIGH_CHAMBER = 0.0;
    public static final double GRAB_OFF_WALL = 0.0;
    public static double kP = 0.032;
    public static double kI = 0.003;
    public static double kD = 0.0002;
    public static double kF = 0.0;
    public static final double ticksInDegrees = 0.403;
    private static PIDController outtakePID;
    private final DcMotor left; //we can make left our lead motor in this case
    private final DcMotor right;
    public boolean slidesReached;
    private double targetPosition = 0.0;
    private boolean manualMode;
    private double power = 0;

    public OuttakeSlides(DcMotor left, DcMotor right, boolean manualMode) {
        this.left = left;
        this.right = right;
        this.manualMode = manualMode;
        setManualMode(manualMode);
        outtakePID = new PIDController(kP, kI, kD);
        outtakePID.setTolerance(3);
    }

    public double getTargetPosition() {
        return targetPosition;
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
            if (targetPosition == 0 && getCurrentPosition() < 100) {
                setSlidesPower(-0.3);
            } else {
                outtakePID.setPID(kP, kI, kD);
                power = outtakePID.calculate(left.getCurrentPosition(), targetPosition);
                slidesReached = (targetPosition > 0 && outtakePID.atSetPoint()) || (left.getCurrentPosition() <= 5 && targetPosition == 0);
//                double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * kF;
                setSlidesPower(power + kF);
            }
        }
        TelemetryUtil.addData("Outtake slides power", power);
    }

    public double getPower() {
        return power;
    }

    public double getCurrentPosition() {
        return left.getCurrentPosition();
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
