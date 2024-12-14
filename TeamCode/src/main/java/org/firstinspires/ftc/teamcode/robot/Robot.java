package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Claw;

public class Robot {
    public final OuttakeArm outtakeArm;
    public final Claw claw;
    //public final Slides outtakeSlides;

    public final DcMotor frontLeftMotor;
    public final DcMotor backLeftMotor;
    public final DcMotor frontRightMotor;
    public final DcMotor backRightMotor;

    public static Follower follower;

    public Robot(HardwareMap hardwareMap) {
        assert RobotConstants.mode != null;
        assert RobotConstants.mode == RobotConstants.Mode.TESTING || RobotConstants.alliance != null;

        // Initialize drive train
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize outtake
        outtakeArm = new OuttakeArm(hardwareMap);
        claw = new Claw(hardwareMap);
        //outtakeSlides = new Slides(hardwareMap);
        outtakeArm.init();
        claw.init();
        //outtakeSlides.init();
    }

    public void update() {
        RobotConstants.START_LOOP();
        outtakeArm.update();
        claw.update();
        //outtakeSlides.update();
    }
}
