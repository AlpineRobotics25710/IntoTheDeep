package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
@TeleOp
public class IntakeArmTest extends CommandOpMode {
    private IntakeArm intakeArm;

    @Override
    public void initialize() {
        Servo armServoRight = hardwareMap.get(Servo.class, "iArmRight");
        Servo armServoLeft = hardwareMap.get(Servo.class, "iArmLeft");
        Servo wristServoRight = hardwareMap.get(Servo.class, "iWristRight");
        Servo wristServoLeft = hardwareMap.get(Servo.class, "iWristLeft");

        armServoRight.setDirection(Servo.Direction.REVERSE);
        armServoLeft.setDirection(Servo.Direction.FORWARD);
        wristServoRight.setDirection(Servo.Direction.FORWARD);
        wristServoLeft.setDirection(Servo.Direction.REVERSE);

        //intakeArm = new IntakeArm(armServoRight, armServoLeft, wristServoRight, wristServoLeft);
        register(intakeArm);

        InstantCommand armIntakeCommand = new InstantCommand(() -> intakeArm.setState(IntakeArm.IntakeArmState.INTAKE));
        InstantCommand armTransferCommand = new InstantCommand(() -> intakeArm.setState(IntakeArm.IntakeArmState.TRANSFER));

        GamepadEx gp1 = new GamepadEx(gamepad1);
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(armIntakeCommand);
        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(armTransferCommand);
    }

    @Override
    public void run() {
        super.run();
    }
}
