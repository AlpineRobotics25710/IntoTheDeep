package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.robot.utils.pids.PIDrip;

@Config
public class Slides implements Mechanism {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private final PIDrip pid;
    private double targetPosition;
    private final double INIT_POS = 0;

    public double Kp;
    public double Ki;
    public double Kd;
    public double Kf;

    public Slides(HardwareMap hardwareMap) {
        this.leftMotor = hardwareMap.dcMotor.get("slidesLeftMotor");
        this.rightMotor = hardwareMap.dcMotor.get("slidesRightMotor");
        this.pid = new PIDrip(Kp, Ki, Kd, Kf, INIT_POS);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        pid.setReference(targetPosition);
    }

    public void moveSlides(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        setTargetPosition(getEncoderPosition());
    }

    @Override
    public void init() {
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
        leftMotor.setPower(pid.updatePID(leftMotor.getCurrentPosition()));
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getEncoderPosition() {
        return leftMotor.getCurrentPosition();
    }
}
