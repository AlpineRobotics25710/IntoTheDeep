package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.robot.utils.PID;

@Config
public class Extendo implements Mechanism {
    private HardwareMap hardwareMap;
    private DcMotor leftMotor, rightMotor;
    private PID pid;
    private double targetPosition;

    public Extendo(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public Extendo(HardwareMap hardwareMap, double targetPosition) {
        this.hardwareMap = hardwareMap;
        this.targetPosition = targetPosition;
    }

    @Override
    public void init() {
        // Initialize the motors
        leftMotor = hardwareMap.get(DcMotor.class, "extendoLeft");
        rightMotor = hardwareMap.get(DcMotor.class, "extendoRight");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the PID controller
        pid = new PID(0.00, 0.0, 0.0, 0.0, targetPosition);
    }

    @Override
    public void update() {
        // Update the PID controller
        pid.setReference(targetPosition);
        double power = pid.updatePID(leftMotor.getCurrentPosition());
        moveExtendo(power);
    }

    public void moveExtendo(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}
