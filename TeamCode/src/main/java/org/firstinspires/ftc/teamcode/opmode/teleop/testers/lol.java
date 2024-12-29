package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.ExtendoSlides;

@TeleOp
public class lol extends LinearOpMode {
    @Override
    public void runOpMode() {
        ExtendoSlides extendoSlides = new ExtendoSlides();
        //extendoSlides.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            extendoSlides.setSlidesPower(gamepad1.left_stick_y);
            telemetry.addData("power", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
