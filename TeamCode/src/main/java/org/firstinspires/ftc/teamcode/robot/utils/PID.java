package org.firstinspires.ftc.teamcode.robot.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double Kp;
    public double Ki;
    public double Kd;
    public double Kf;
    private final ElapsedTime timer = new ElapsedTime();
    private double reference;
    private double motorPower;
    private double lastError;
    private double integralSum = 0;

    public PID(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    public PID(double Kp, double Ki, double Kd, double Kf, double reference) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.reference = reference;
    }

    public void setReference(double reference) {
        this.reference = reference;
    }

    public double getMotorPower() {
        return motorPower;
    }

    public double updatePID(double encoderPosition) {
        // calculate the error
        double error = reference - encoderPosition;

        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();
        timer.reset();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + Kf;

        lastError = error;
        motorPower = out;

        return out;
    }

    public void resetTimer() {
        this.timer.reset();
    }
}
