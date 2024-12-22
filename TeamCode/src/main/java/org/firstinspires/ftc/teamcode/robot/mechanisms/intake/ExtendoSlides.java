package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.MechanismState;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Slides;

@Config
public class ExtendoSlides extends Slides {
    public static double BASE_POS = 0.0;
    public static double TRANSFER_POS = 0.0;
    public static double INTAKE_POS = 0.0;

    public static double Kp;
    public static double Ki;
    public static double Kd;
    public static double Kf;

    private final HardwareMap hardwareMap;

    public ExtendoSlides(HardwareMap hardwareMap) {
        super(Kp, Ki, Kd, Kf);
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        this.leftMotor = hardwareMap.dcMotor.get("extendoLeftMotor");
        this.rightMotor = hardwareMap.dcMotor.get("extendoRightMotor");

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

    @Override
    public void ascend() {
        setTargetPosition(INTAKE_POS);
        slidesState = MechanismState.ASCENT;
    }

    @Override
    public void transfer() {
        setTargetPosition(TRANSFER_POS);
        slidesState = MechanismState.TRANSFER;
    }

    @Override
    public void intake() {
        setTargetPosition(INTAKE_POS);
        slidesState = MechanismState.INTAKE;
    }

    @Override
    public void lowChamber() {
        setTargetPosition(BASE_POS);
        slidesState = MechanismState.LOW_CHAMBER;
    }

    @Override
    public void highChamber() {
        setTargetPosition(BASE_POS);
        slidesState = MechanismState.HIGH_CHAMBER;
    }

    @Override
    public void lowBasket() {
        setTargetPosition(BASE_POS);
        slidesState = MechanismState.LOW_BASKET;
    }

    @Override
    public void highBasket() {
        setTargetPosition(BASE_POS);
        slidesState = MechanismState.HIGH_BASKET;
    }
}
