package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@TeleOp
@Config
public class ExtendoValueGetter extends LinearOpMode {
    public static double targetPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        DcMotor extendoRight = hardwareMap.get(DcMotor.class, "extendoRight");
        extendoRight.setDirection(DcMotor.Direction.REVERSE);
        extendoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Extendo extendo = new Extendo(extendoRight, false);
        //extendoRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            extendo.setTargetPosition(targetPosition);
            extendo.periodic();
            TelemetryUtil.update();
        }
    }
}
