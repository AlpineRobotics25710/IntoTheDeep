package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.utils.PID;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public abstract class Slides extends SubsystemBase implements Mechanism {
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;

    protected final PID pid;
    public double targetPosition;

    public boolean manualMode = false;


    public Slides(double Kp, double Ki, double Kd, double Kf) {
        this.pid = new PID(Kp, Ki, Kd, Kf);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        pid.setReference(targetPosition);
    }

    public void moveSlides(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void setManualMode(boolean manualMode) {
        this.manualMode = manualMode;
        if(manualMode) {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void periodic() {
        if (!manualMode) { // Only use PID control if not in manual mode
            double power = pid.updatePID(leftMotor.getCurrentPosition());
            moveSlides(power);
        }
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getEncoderPosition() {
        return leftMotor.getCurrentPosition();
    }
}
