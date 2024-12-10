package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.utils.control.gamepad.CustomGamepad;
import org.firstinspires.ftc.teamcode.robot.utils.control.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.utils.control.drivetrain.DrivetrainBuilder;

@TeleOp
public class OuttakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this.hardwareMap);
        RobotConstants.mode = RobotConstants.Mode.TESTING;

        CustomGamepad gp1 = new CustomGamepad(gamepad1);
        gp1.getLeftStick().setSensitivity(1.0);
        gp1.getRightStick().setSensitivity(1.0);
        gp1.getA().setAction(() -> gp1.getA().isClicked(), () -> robot.getClaw().open());
        gp1.getB().setAction(() -> gp1.getB().isClicked(), () -> robot.getClaw().close());
        gp1.getX().setAction(() -> gp1.getX().isClicked(), () -> robot.getSlides().setTargetPosition(robot.getSlides().getTargetPosition() + 10));
        gp1.getY().setAction(() -> gp1.getY().isClicked(), () -> robot.getSlides().setTargetPosition(robot.getSlides().getTargetPosition() - 10));

        Drivetrain drivetrain = new DrivetrainBuilder()
                .setType(DrivetrainBuilder.DrivetrainType.ROBOT_CENTRIC_MECANUM)
                .setGamepad(gp1)
                .setMotors(robot.frontLeftMotor, robot.backLeftMotor, robot.frontRightMotor, robot.backRightMotor)
                .setStrafingMultiplier(1.1)
                .build();

        while(opModeInInit()) {
            robot.getSlides().update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double manualSlidesPower = Math.pow(gp1.getLeftStick().getY(), 3) / 1.5;
            robot.getSlides().moveSlides(manualSlidesPower);

            drivetrain.update();
            gp1.update();
            robot.update();

            telemetry.addData("Left stick y: ", -gamepad1.left_stick_y);
            telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
