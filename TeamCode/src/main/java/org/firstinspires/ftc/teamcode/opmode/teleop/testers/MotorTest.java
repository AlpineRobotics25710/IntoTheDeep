package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
@TeleOp
public class MotorTest extends OpMode {
    public static double motorPower = 0.1;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    int selectedMotorIndex = 0;
    DcMotor[] motors;
    String[] motorNames = {"backRightMotor", "backLeftMotor", "frontRightMotor", "frontLeftMotor"};

    @Override
    public void init() {
        backRightMotor = hardwareMap.get(DcMotor.class, motorNames[0]);
        backLeftMotor = hardwareMap.get(DcMotor.class, motorNames[1]);
        frontRightMotor = hardwareMap.get(DcMotor.class, motorNames[2]);
        frontLeftMotor = hardwareMap.get(DcMotor.class, motorNames[3]);

        motors = new DcMotor[]{backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor};
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

        TelemetryUtil.addData("Selected Motor", motorNames[selectedMotorIndex]);
        TelemetryUtil.addData("Motor Power", motorPower);
        TelemetryUtil.update();
    }
}
