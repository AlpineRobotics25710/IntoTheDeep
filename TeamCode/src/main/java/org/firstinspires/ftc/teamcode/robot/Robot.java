package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;

public class Robot {
    public OuttakeArm outtakeArm;
    public OuttakeClaw outtakeClaw;
    //public Slides outtakeSlides;
    public IntakeArm intakeArm;
    public IntakeClaw intakeClaw;

    public final DcMotor frontLeftMotor;
    public final DcMotor backLeftMotor;
    public final DcMotor frontRightMotor;
    public final DcMotor backRightMotor;

    public static Follower follower;

    public Robot(HardwareMap hardwareMap) {
        //assert RobotConstants.mode != null;
        //assert RobotConstants.mode == RobotConstants.Mode.TESTING || RobotConstants.alliance != null;

        // Initialize drive train
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize outtake
        //outtakeArm = new OuttakeArm(hardwareMap);
        //outtakeClaw = new OuttakeClaw(hardwareMap);
        //outtakeSlides = new Slides(hardwareMap);
        //outtakeArm.init();
        //outtakeClaw.init();
        //outtakeSlides.init();

        // Initialize intake
        intakeArm = new IntakeArm(hardwareMap);
        intakeClaw = new IntakeClaw(hardwareMap);
        intakeArm.init();
        intakeClaw.init();
    }

    public void update() {
        RobotConstants.START_LOOP();
        //outtakeArm.update();
        //outtakeClaw.update();
        //outtakeSlides.update();
    }
}
