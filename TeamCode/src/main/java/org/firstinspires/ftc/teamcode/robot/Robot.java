package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Arm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Claw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Slides;
import org.firstinspires.ftc.teamcode.robot.utils.wrappers.HardwareQueue;
import org.firstinspires.ftc.teamcode.robot.sensors.Sensors;

public class Robot {
    private final HardwareQueue hardwareQueue;
    private final Sensors sensors;
    private final Arm arm;
    private final Claw claw;
    private final Slides slides;
    public final DcMotor frontLeftMotor;
    public final DcMotor backLeftMotor;
    public final DcMotor frontRightMotor;
    public final DcMotor backRightMotor;

    public Robot(HardwareMap hardwareMap) {
        hardwareQueue = new HardwareQueue();

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
        slides = new Slides(hardwareMap);
        arm.init();
        claw.init();
        slides.init();

        // Initialize sensors
        sensors = new Sensors(hardwareMap, hardwareQueue, this);
        sensors.init();
    }

    public void update() {
        RobotConstants.START_LOOP();
        hardwareQueue.update();
        sensors.update();
        arm.update();
        claw.update();
        slides.update();
    }

    public HardwareQueue getHardwareQueue() {
        return hardwareQueue;
    }

    public Arm getArm() {
        return arm;
    }

    public Claw getClaw() {
        return claw;
    }

    public Slides getSlides() {
        return slides;
    }

    public Sensors getSensors() {
        return sensors;
    }
}
