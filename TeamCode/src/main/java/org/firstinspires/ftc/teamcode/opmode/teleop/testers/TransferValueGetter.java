package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import static org.firstinspires.ftc.teamcode.opmode.teleop.testers.OuttakeValueGetter.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@TeleOp
@Config
public class TransferValueGetter extends LinearOpMode {
    public static double outtakeArmPos = 0.0;
    public static double outtakeWristPos = 0.0;
    public static double outtakeClawPos = 0.0;
    public static double outtakeSwivelPos = 0.0;
    public static double intakeArmPos = 0.0;
    public static double intakeWristPos = 0.0;
    public static IntakeEnd.ActiveState activeState = IntakeEnd.ActiveState.OFF;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Servo oArmLeft = hardwareMap.get(Servo.class, "oArmLeft");
        Servo oArmRight = hardwareMap.get(Servo.class, "oArmRight");
        Servo oWrist = hardwareMap.get(Servo.class, "oWrist");

        Servo outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");
        Servo outtakeSwivelServo = hardwareMap.get(Servo.class, "outtakeSwivelServo");

        oArmRight.setDirection(Servo.Direction.REVERSE);
        oWrist.setDirection(Servo.Direction.REVERSE);

        OuttakeArm outtakeArm = new OuttakeArm(oArmRight, oArmLeft, oWrist);
        OuttakeClaw outtakeClaw = new OuttakeClaw(outtakeClawServo, outtakeSwivelServo);

        CRServo activeIntake = hardwareMap.get(CRServo.class, "activeIntake");
        Servo iArmLeft = hardwareMap.get(Servo.class, "iArmLeft");
        Servo iArmRight = hardwareMap.get(Servo.class, "iArmRight");
        Servo iWristRight = hardwareMap.get(Servo.class, "iWristRight");

        iArmRight.setDirection(Servo.Direction.REVERSE);

        IntakeArm intakeArm = new IntakeArm(iArmRight, iArmLeft, iWristRight);
        IntakeEnd intakeEnd = new IntakeEnd(activeIntake);

        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendoRight");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

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
            extendo.setPower(-0.2);
            outtakeArm.setArmPosition(outtakeArmPos);
            outtakeArm.setWristPosition(outtakeWristPos);
            outtakeClaw.setClawPosition(outtakeClawPos);
            outtakeClaw.setSwivelPosition(outtakeSwivelPos);
            intakeArm.setArmPosition(intakeArmPos);
            intakeArm.setWristPosition(intakeWristPos);
            intakeEnd.setState(activeState);

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
