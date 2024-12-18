package org.firstinspires.ftc.teamcode.opmode.auto.timebased;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Time Based Park Only Red Observation Side")
public class RedObservationSide extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        strafe(1000, 0.75);
    }

    public void strafe(long time, double maxPower) throws InterruptedException {
        long halfTime = time / 2;
        accelerate(halfTime, maxPower);
        decelerate(halfTime, maxPower);
    }

    public void accelerate(long time, double maxPower) {
        long interval = 50; // Time interval to increment power
        int steps = (int) (time / interval);
        for (int i = 0; i < steps; i++) {
            double power = maxPower * (i / (double) steps);
            setPowerForStrafing(power);
            sleep(interval);
        }
    }

    public void decelerate(long time, double maxPower) {
        long interval = 50; // Time interval to decrement power
        int steps = (int) (time / interval);
        for (int i = 0; i < steps; i++) {
            double power = maxPower * (1 - (i / (double) steps));
            setPowerForStrafing(power);
            sleep(interval);
        }
        stopAll();
    }

    public void setPowerForStrafing(double power) {
        power = Range.clip(power, -1, 1);
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(power);
    }

    public void stopAll() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
