package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class LibExtendoTest extends LinearOpMode {
    public static double posCoefficient = 0.02;
    public static int targetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Motor extendoMotor = new Motor(hardwareMap, "extendoRight", Motor.GoBILDA.RPM_60);
        extendoMotor.setRunMode(Motor.RunMode.PositionControl);
        extendoMotor.setPositionCoefficient(posCoefficient);

        while (opModeIsActive()) {
            extendoMotor.setTargetPosition(targetPosition);
            extendoMotor.setInverted(true);
            extendoMotor.setPositionTolerance(5.0);
            while (!extendoMotor.atTargetPosition()) {
                extendoMotor.set(0.75);
            }
            extendoMotor.stopMotor();
        }
    }
}
