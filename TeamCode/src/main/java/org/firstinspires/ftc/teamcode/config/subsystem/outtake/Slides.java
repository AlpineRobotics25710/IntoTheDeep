package org.firstinspires.ftc.teamcode.config.subsystem.outtake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.utils.Globals;
import org.firstinspires.ftc.teamcode.config.utils.RunMode;
import org.firstinspires.ftc.teamcode.config.utils.Utils;
import org.firstinspires.ftc.teamcode.config.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.config.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.config.sensors.Sensors;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class Slides {
    private final PriorityMotor slidesMotors;
    public double length;
    public double vel;
    private final Sensors sensors;
    public static double ticksToInches = 0.04132142857142857;
    public static double maxSlidesLength = 10;
    private double targetLength = 0;
    public static double maxVel = 1.6528571428571428;
    public static double kP = 0.15; // used to be 0.11
    public static double kA = 3;
    public static double kStatic = 0.15;
    public static double minPower = 0.2850000000000002;
    public static double minPowerThresh = 0.8;
    public double downPower = -0.1;
    public static double forceDownPower = -0.5;

    public PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static double ticks_in_degree = 700 / 180.0;
    private DcMotorEx m1;
    private DcMotorEx m2;

    public Slides(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.sensors = sensors;

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        m1 = hardwareMap.get(DcMotorEx.class, "slidesMotor1");
        m2 = hardwareMap.get(DcMotorEx.class, "slidesMotor2");
        DcMotorEx[] motors = {m1, m2};
        m2.setDirection(DcMotorSimple.Direction.REVERSE); //might have to be changed

        if (Globals.mode != RunMode.TELEOP) {
            resetExtendoEncoders();
        }

        slidesMotors = new PriorityMotor(motors, "slidesMotor", 2, 5, new double[]{1, 1}, sensors);
        hardwareQueue.addDevice(slidesMotors);
    }

    public void resetExtendoEncoders() {
        Log.e("RESETTTING", "RESTETING MOTOR *************");

        //drivetrain.resetSlidesMotorRightFront();

        m1.setPower(0);
        m2.setPower(0);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetLength = 0;
        m1.setPower(0);
        m2.setPower(0);
    }

    public void setSlidesMotorsToCoast() {
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setSlidesMotorsToBrake() {
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * @return power
     */
    private double power() {
        controller.setPID(p, i, d);

        //we need an f
        double slidesPos = sensors.getSlidesPos();
        double pid = controller.calculate(slidesPos, targetLength * ticksToInches);
        double power = pid + f;

        telemetry.addData("slides pos ", slidesPos);
        telemetry.addData("target ", targetLength * ticksToInches);
        return power;
    }

    public boolean manualMode = false;
    public void update() {
        length = (double) sensors.getExtendoPos() * ticksToInches;
        vel = sensors.getExtendoVel() * ticksToInches;

        if (!manualMode) {
//            if (!(Globals.RUNMODE == RunMode.TESTER)) {
            double power = power();
            slidesMotors.setTargetPower(Utils.clip(power));
//            }
        }
    }

    public void setTargetLength(double length) {
        targetLength = Math.max(Math.min(length, maxSlidesLength),0);
    }

    public void setTargetPowerFORCED(double power) {
        slidesMotors.setTargetPower(Math.max(Math.min(power, 1), -1));
    }

    public void turnOffPowerFORCED() {
        slidesMotors.motor[0].setPower(0);
        slidesMotors.motor[1].setPower(0);
    }

    public void setTargetLengthFORCED(double length) {
        targetLength = length;
    }

    public boolean inPosition(double threshold) {
        return Math.abs(targetLength - length) <= threshold;
    }

    public double getLength(double length) {
        return this.length;
    }

    public double getLength() {
        return length;
    }
}