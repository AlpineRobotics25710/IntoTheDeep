package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;

@TeleOp
public class OuttakeSlidesValueGetter extends LinearOpMode {
    public static double targetPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor outtakeSlideLeft = hardwareMap.get(DcMotor.class, "outtakeLeft");
        DcMotor outtakeSlideRight = hardwareMap.get(DcMotor.class, "outtakeRight");

        outtakeSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeSlideRight.setDirection(DcMotor.Direction.FORWARD);
        outtakeSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeSlideRight.setDirection(DcMotor.Direction.FORWARD);

        Extendo extendo = new Extendo(outtakeSlideLeft, false);

        waitForStart();

        while (opModeIsActive()) {
            extendo.setTargetPosition(targetPosition);
            extendo.periodic();
        }
    }
}
