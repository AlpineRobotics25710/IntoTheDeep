package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
@TeleOp
public class MotorTest extends OpMode {
    public static double motorPower = 0.0;

    // Declare motor references
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;

    // Variable to track which motor is currently selected
    int selectedMotorIndex = 0;
    DcMotor[] motors;
    String[] motorNames = {"backRightMotor", "backLeftMotor", "frontRightMotor", "frontLeftMotor"};

    @Override
    public void init() {
        // Initialize motors
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");

        motors = new DcMotor[]{backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor};

        // Set up motors
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        TelemetryUtil.setup(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            selectedMotorIndex = (selectedMotorIndex + 1) % motors.length;
        } else if (gamepad1.dpad_down) {
            selectedMotorIndex = (selectedMotorIndex - 1 + motors.length) % motors.length;
        }

        for (int i = 0; i < motors.length; i++) {
            if (i == selectedMotorIndex) {
                motors[i].setPower(motorPower);
            } else {
                motors[i].setPower(0);
            }
        }

        // Display telemetry data
        TelemetryUtil.addData("Selected Motor", motorNames[selectedMotorIndex]);
        TelemetryUtil.addData("Motor Power", motorPower);
        TelemetryUtil.update();
    }
}
