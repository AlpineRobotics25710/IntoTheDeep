package org.firstinspires.ftc.teamcode.teleop.testers.gamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.utils.control.gamepad.CustomGamepad;
import org.firstinspires.ftc.teamcode.robot.utils.control.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.utils.control.drivetrain.DrivetrainBuilder;

@TeleOp
public class DrivetrainTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        CustomGamepad gp1 = new CustomGamepad(gamepad1);
        gp1.getLeftStick().setSensitivity(1.0);
        gp1.getRightStick().setSensitivity(1.0);
        gp1.getA().setAction(() -> gp1.getA().isClicked(), () -> telemetry.addData("A is clicked", "through lambda"));
        gp1.getB().setAction(() -> gp1.getB().isClicked(), () -> telemetry.addData("B is clicked", "through lambda"));

        Drivetrain drivetrain = new DrivetrainBuilder()
                .setType(DrivetrainBuilder.DrivetrainType.ROBOT_CENTRIC_MECANUM)
                .setGamepad(gp1)
                .setMotors(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)
                .setStrafingMultiplier(1.1)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drivetrain.update();
            gp1.update();

            telemetry.addData("Left stick y: ", -gamepad1.left_stick_y);
            telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
