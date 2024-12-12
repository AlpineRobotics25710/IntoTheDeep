package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.robot.utils.PID;

@Config
public class Slides implements Mechanism {
    private final HardwareMap hardwareMap;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private final PID pid;
    public double targetPosition;
    public final double INIT_POS = 0;

    public double Kp;
    public double Ki;
    public double Kd;
    public double Kf;

    public boolean manualMode = false;

    public Slides(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.pid = new PID(Kp, Ki, Kd, Kf, INIT_POS);
    }

    public void setTargetPosition(double targetPosition) {
        if (!manualMode) {  // Only set target position if not in manual mode
            this.targetPosition = targetPosition;
            pid.setReference(targetPosition);
        }
    }

    public void moveSlides(double power) {
        if (manualMode) {
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        } else {
            setTargetPosition(getEncoderPosition());
        }
    }

    public void setManualMode(boolean manualMode) {
        this.manualMode = manualMode;
        if (manualMode) {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Allow free control of power
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setTargetPosition(getEncoderPosition()); // Reinitialize PID target when exiting manual mode
        }
    }

    @Override
    public void init() {
        this.leftMotor = hardwareMap.dcMotor.get("slidesLeftMotor");
        this.rightMotor = hardwareMap.dcMotor.get("slidesRightMotor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setTargetPosition(INIT_POS);
    }

    @Override
    public void update() {
        if (!manualMode) { // Only use PID control if not in manual mode
            double power = pid.updatePID(leftMotor.getCurrentPosition());
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getEncoderPosition() {
        return leftMotor.getCurrentPosition();
    }
}
