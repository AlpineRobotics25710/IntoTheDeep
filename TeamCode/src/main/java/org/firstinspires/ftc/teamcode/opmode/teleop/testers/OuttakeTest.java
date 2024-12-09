package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.control.gamepad.CustomGamepad;
import org.firstinspires.ftc.teamcode.robot.utils.control.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.utils.control.drivetrain.DrivetrainBuilder;

@TeleOp
public class OuttakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this.hardwareMap);

        CustomGamepad gp1 = new CustomGamepad(gamepad1);
        gp1.getLeftStick().setSensitivity(1.0);
        gp1.getRightStick().setSensitivity(1.0);
        gp1.getA().setAction(() -> gp1.getA().isClicked(), () -> robot.getOuttake().getClaw().open());
        gp1.getB().setAction(() -> gp1.getB().isClicked(), () -> robot.getOuttake().getClaw().close());

        Drivetrain drivetrain = new DrivetrainBuilder()
                .setType(DrivetrainBuilder.DrivetrainType.ROBOT_CENTRIC_MECANUM)
                .setGamepad(gp1)
                .setMotors(robot.frontLeftMotor, robot.backLeftMotor, robot.frontRightMotor, robot.backRightMotor)
                .setStrafingMultiplier(1.1)
                .build();

        while(opModeInInit()) {
            // TODO: Get rid of outtake wrapper class and just add a transferOuttake() method to the Robot class
            robot.getOuttake().getSlides().update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drivetrain.update();
            gp1.update();
            robot.update();

            telemetry.addData("Left stick y: ", -gamepad1.left_stick_y);
            telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
