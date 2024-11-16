package org.firstinspires.ftc.teamcode.robot.testers.gamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.utils.gamepad.drivetrain.RobotCentricMecanumDrivetrain;

public class DrivetrainTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RobotCentricMecanumDrivetrain drivetrain = new RobotCentricMecanumDrivetrain(frontLeftMotor,
                backLeftMotor, frontRightMotor, backRightMotor, gamepad1);

        drivetrain.init();

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            drivetrain.update();
        }
    }
}
