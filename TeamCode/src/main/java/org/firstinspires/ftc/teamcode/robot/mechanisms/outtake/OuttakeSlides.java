package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Slides;

@Config
public class OuttakeSlides extends Slides {
    public static double BASE_POS = 0.0;
    public static double LOW_BASKET_POS = 0.0;
    public static double HIGH_BASKET_POS = 0.0;
    public static double LOW_RUNG_POS = 0.0;
    public static double HIGH_RUNG_POS = 0.0;
    public static double TRANSFER_POS = 0.0;

    public static double Kp;
    public static double Ki;
    public static double Kd;
    public static double Kf;

    public OuttakeSlides() {
        super(Kp, Ki, Kd, Kf);
        this.leftMotor = robot.outtakeSlideLeft;
        this.rightMotor = robot.outtakeSlideRight;
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

        setTargetPosition(BASE_POS);
    }
}
