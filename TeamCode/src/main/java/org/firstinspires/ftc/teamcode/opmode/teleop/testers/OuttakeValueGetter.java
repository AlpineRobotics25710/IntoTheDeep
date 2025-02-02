package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static boolean drive = false;
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

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            outtakeArm.setArmPosition(armPos);
            outtakeArm.setWristPosition(wristPos);
            outtakeClaw.setClawPosition(clawPos);
            outtakeClaw.setSwivelPosition(swivelPos);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            if(drive) {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }
            TelemetryUtil.update();
        }
    }
}
