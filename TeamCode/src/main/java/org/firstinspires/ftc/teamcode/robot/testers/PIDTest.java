package org.firstinspires.ftc.teamcode.robot.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.utils.gamepad.CustomGamepad;

@TeleOp(group = "Testers")
@Config
public class PIDTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CustomGamepad gp1 = new CustomGamepad(this.gamepad1);

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
            // Extendo code with custom gamepad code
            if(gp1.getA().isClicked()) {
                extendo.setTargetPosition(extendo.getTargetPosition() + 10);
            }

            if(gp1.getY().isClicked()) {
                extendo.setTargetPosition(extendo.getTargetPosition() - 10);
            }
            
            extendo.update();
            gp1.update();
        }
    }
}