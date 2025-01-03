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
    DcMotor test;

    public void init(){
        test = hardwareMap.get(DcMotor.class, "backRightMotor");
        TelemetryUtil.setup(telemetry);
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        test.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void loop(){
        test.setPower(motorPower);
        TelemetryUtil.addData("Motor Power", motorPower);
        TelemetryUtil.addData("Current Position", test.getCurrentPosition());
        TelemetryUtil.update();
    }
}
