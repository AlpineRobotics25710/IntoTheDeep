package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class AnothaTest extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = Robot.getInstance();
        robot.init(hardwareMap);  // Initialize the robot and its motors
    }

    @Override
    public void loop() {
        double power = -gamepad1.left_stick_y;  // Use gamepad to control power
        robot.extendoLeft.setPower(power);
        robot.extendoRight.setPower(power);
    }
}
