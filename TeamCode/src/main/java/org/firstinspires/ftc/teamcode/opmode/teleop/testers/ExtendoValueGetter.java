package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;

@TeleOp
public class ExtendoValueGetter extends LinearOpMode {
    public static double targetPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor extendoLeft = hardwareMap.get(DcMotor.class, "extendoLeft");
        DcMotor extendoRight = hardwareMap.get(DcMotor.class, "extendoRight");

        extendoLeft.setDirection(DcMotor.Direction.REVERSE);
        extendoRight.setDirection(DcMotor.Direction.FORWARD);

        Extendo extendo = new Extendo(extendoLeft, false);

        waitForStart();

        while (opModeIsActive()) {
            extendo.setTargetPosition(targetPosition);
            extendo.periodic();
        }
    }
}
