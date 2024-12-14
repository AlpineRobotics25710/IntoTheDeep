package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClaw;

@Config
@TeleOp
public class IntakeTest extends LinearOpMode {
    public static double armPos = 0.0;
    public static double wristPos = 0.0;
    public static double clawPos = 0.0;
    public static double swivelPos = 0.0;


    @Override
    public void runOpMode() {
        IntakeArm intakeArm = new IntakeArm(hardwareMap);
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap);
        intakeArm.init();
        intakeClaw.init();

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            intakeArm.setArmPosition(armPos);
            intakeArm.setWristPosition(wristPos);
            intakeClaw.setClawPosition(clawPos);
            intakeClaw.setSwivelPosition(swivelPos);
            telemetry.addData("Arm pos", intakeArm.getArmPosition());
            telemetry.addData("Wrist pos", wristPos);
            telemetry.addData("Claw pos", clawPos);
            telemetry.addData("Swivel pos", swivelPos);
            telemetry.update();
        }
    }
}
