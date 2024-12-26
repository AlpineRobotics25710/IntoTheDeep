package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Slides;

@Config
public class ExtendoSlides extends Slides {
    Robot robot = Robot.getInstance();
    public static double BASE_POS = 0.0;
    public static double TRANSFER_POS = 0.0;
    public static double INTAKE_POS = 0.0;

    public static double Kp;
    public static double Ki;
    public static double Kd;
    public static double Kf;
    public ExtendoSlides() {
        super(Kp, Ki, Kd, Kf);
    }

    @Override
    public void init() {
        this.leftMotor = robot.extendoLeft;
        this.rightMotor = robot.extendoRight;

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setTargetPosition(BASE_POS);
    }
}
