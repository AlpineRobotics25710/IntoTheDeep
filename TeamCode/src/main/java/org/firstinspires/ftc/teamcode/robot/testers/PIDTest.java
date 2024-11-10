package org.firstinspires.ftc.teamcode.robot.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;

@TeleOp
public class PIDTest extends LinearOpMode {
    private boolean gamepadA;
    private boolean gamepadY;

    @Override
    public void runOpMode() throws InterruptedException {
        resetFlags();

        Extendo extendo = new Extendo(this.hardwareMap, 0.0);
        extendo.init();
        telemetry.addData("Intake Status", "Initialized");

        telemetry.update();

        while (opModeInInit()) {
            if (isStopRequested()) return;
            extendo.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // Extendo code
            if (gamepad1.a && !gamepadA) {
                extendo.setTargetPosition(extendo.getTargetPosition() + 10);
                gamepadA = true;
            }

            if (gamepad1.y && !gamepadY) {
                extendo.setTargetPosition(extendo.getTargetPosition() - 10);
                gamepadY = true;
            }
        }
    }

    // Resetting the flags
    public void resetFlags() {
        if (!gamepad1.a) {
            gamepadA = false;
        }

        if (!gamepad1.y) {
            gamepadY = false;
        }
    }
}