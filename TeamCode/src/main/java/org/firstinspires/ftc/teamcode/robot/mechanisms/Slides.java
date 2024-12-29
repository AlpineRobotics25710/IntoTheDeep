package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.utils.PID;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public abstract class Slides extends SubsystemBase implements Mechanism {
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;

    protected final PID pid;
    public double targetPosition;

    public static boolean manualMode;
    private double manualPower = 0.0;

    public Slides(double Kp, double Ki, double Kd, double Kf) {
        this.pid = new PID(Kp, Ki, Kd, Kf);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        pid.setReference(targetPosition);
    }

    protected void moveSlides(double power) {
        TelemetryUtil.packet.put("moving motors in move slides method at power", power);
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

    public void setManualPower(double manualPower) {
        this.manualPower = manualPower;
    }

    @Override
    public void periodic() {
        TelemetryUtil.packet.put("manual mode", manualMode);
        if (!manualMode) { // Only use PID control if not in manual mode
            TelemetryUtil.packet.put("In", "manual mode if statement");
            double power = pid.updatePID(leftMotor.getCurrentPosition());
            moveSlides(power);
        } else {
            TelemetryUtil.packet.put("In", "else of manual mode if statement");
            moveSlides(manualPower);
        }
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getEncoderPosition() {
        return leftMotor.getCurrentPosition();
    }
}
