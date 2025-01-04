package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;

@Config
@TeleOp
public class IntakeValueGetter extends LinearOpMode {
    public static double armPos;
    public static double wristPos;
    public static IntakeEnd.ActiveState activeState;

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo activeIntake = hardwareMap.get(CRServo.class, "activeIntake");
        Servo iArmLeft = hardwareMap.get(Servo.class, "iArmLeft");
        Servo iArmRight = hardwareMap.get(Servo.class, "iArmRight");
        Servo iWristRight = hardwareMap.get(Servo.class, "iWristRight");

        iArmRight.setDirection(Servo.Direction.REVERSE);
//        iWristLeft.setDirection(Servo.Direction.REVERSE);

       IntakeArm intakeArm = new IntakeArm(iArmRight, iArmLeft, iWristRight);
        IntakeEnd intakeEnd = new IntakeEnd(activeIntake);

        waitForStart();

        while (opModeIsActive()) {
            intakeArm.setArmPosition(armPos);
            intakeArm.setWristPosition(wristPos);
            intakeEnd.setState(activeState);
        }
    }
}
