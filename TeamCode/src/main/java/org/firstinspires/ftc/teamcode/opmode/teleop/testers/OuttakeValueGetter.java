package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
@TeleOp
public class OuttakeValueGetter extends LinearOpMode {
    public static double armPos;
    public static double wristPos;
    public static double clawPos;
    public static double swivelPos;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Servo oArmLeft = hardwareMap.get(Servo.class, "oArmLeft");
        Servo oArmRight = hardwareMap.get(Servo.class, "oArmRight");
        Servo oWrist = hardwareMap.get(Servo.class, "oWrist");

        Servo outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");
        Servo outtakeSwivelServo = hardwareMap.get(Servo.class, "outtakeSwivelServo");

        oArmRight.setDirection(Servo.Direction.REVERSE);

        OuttakeArm outtakeArm = new OuttakeArm(oArmRight, oArmLeft, oWrist);
        OuttakeClaw outtakeClaw = new OuttakeClaw(outtakeClawServo, outtakeSwivelServo);

        waitForStart();

        while (opModeIsActive()) {
            outtakeArm.setArmPosition(armPos);
            outtakeArm.setWristPosition(wristPos);
            outtakeClaw.setClawPosition(clawPos);
            outtakeClaw.setSwivelPosition(swivelPos);
            TelemetryUtil.update();
        }
    }
}
