package org.firstinspires.ftc.teamcode.config.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group="Test")
public class ServoTesters extends OpMode {
    Servo leftArm;
    Servo rightArm;

    Servo leftClaw;
    Servo rightClaw;

    @Override
    public void init() {
        // Initialize the servos using hardwareMap and ensure they are named correctly in your configuration file
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");


    }

    @Override
    public void loop() {
        // Use the gamepad to control the servos for testing
        // Left joystick controls arms
        double armPosition = (gamepad1.left_stick_y + 1) / 2; // Normalize joystick value to 0-1 range
        leftArm.setPosition(armPosition); // Direct movement
        rightArm.setPosition(1 - armPosition);

        // Right joystick controls claws
        double clawPosition = (gamepad1.right_stick_y + 1) / 2; // Normalize joystick value to 0-1 range
        leftClaw.setPosition(clawPosition); // Direct movement
        rightClaw.setPosition(1 - clawPosition); // Reverse movement

        // Display positions on telemetry for debugging
        telemetry.addData("Arm Positions", "Left: %.2f, Right: %.2f", armPosition, 1 - armPosition);
        telemetry.addData("Claw Positions", "Left: %.2f, Right: %.2f", clawPosition, 1 - clawPosition);
        telemetry.update();
    }
}
