package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class goofy extends OpMode {
    DcMotor test;
    DcMotor test2;

    @Override
    public void init() {
        test = hardwareMap.get(DcMotor.class, "extendoLeft");
        test2 = hardwareMap.get(DcMotor.class, "extendoRight");

        test.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        test2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        test.setDirection(DcMotor.Direction.FORWARD);
        test2.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Cubed to slowly increase speed
        double manualPower = -gamepad1.left_stick_y;
        test.setPower(manualPower);
        test2.setPower(manualPower);

        telemetry.addData("manual power", manualPower);
        telemetry.update();
    }
}

