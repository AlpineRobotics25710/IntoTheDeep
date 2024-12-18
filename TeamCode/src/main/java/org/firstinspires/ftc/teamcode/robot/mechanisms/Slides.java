package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.utils.PID;

@Config
public abstract class Slides implements Mechanism {
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;

    protected final PID pid;
    public double targetPosition;

    public boolean manualMode = false;

    protected MechanismState slidesState;

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
    }

    @Override
    public void update() {
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
