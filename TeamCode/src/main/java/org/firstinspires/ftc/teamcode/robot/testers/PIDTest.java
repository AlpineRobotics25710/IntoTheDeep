package org.firstinspires.ftc.teamcode.robot.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;

@TeleOp(group = "Testers")
@Config
public class PIDTest extends LinearOpMode {
    private boolean gamepadA;
    private boolean gamepadY;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
            resetFlags();

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