package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Claw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Slides;
import org.firstinspires.ftc.teamcode.robot.sensors.Sensors;

public class Robot {
    public final Sensors sensors;
    public final Arm arm;
    public final Claw claw;
    public final Slides outtakeSlides;

    public final DcMotor frontLeftMotor;
    public final DcMotor backLeftMotor;
    public final DcMotor frontRightMotor;
    public final DcMotor backRightMotor;

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
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        outtakeSlides = new Slides(hardwareMap);
        arm.init();
        claw.init();
        outtakeSlides.init();

        // Initialize sensors
        sensors = new Sensors(hardwareMap, this);
        sensors.init();
    }

    public void update() {
        RobotConstants.START_LOOP();
        sensors.update();
        arm.update();
        claw.update();
        outtakeSlides.update();
    }
}
