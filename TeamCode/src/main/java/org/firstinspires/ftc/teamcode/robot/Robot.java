package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robot.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.utils.wrappers.HardwareQueue;
import org.firstinspires.ftc.teamcode.robot.sensors.Sensors;

public class Robot {
    private final HardwareQueue hardwareQueue;
    private final Sensors sensors;
    private final Outtake outtake;
    public final DcMotor frontLeftMotor;
    public final DcMotor backLeftMotor;
    public final DcMotor frontRightMotor;
    public final DcMotor backRightMotor;

    public Robot(HardwareMap hardwareMap) {
        hardwareQueue = new HardwareQueue();
        sensors = new Sensors(hardwareMap, hardwareQueue, this);
        outtake = new Outtake(hardwareMap);

        assert RobotConstants.mode != null;
        assert RobotConstants.mode == RobotConstants.Mode.TESTING || RobotConstants.alliance != null;

        // Initialize drive train
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize mechanisms
        outtake.init();
        sensors.init();
    }

    public void update() {
        RobotConstants.START_LOOP();
        hardwareQueue.update();
        sensors.update();
        outtake.update();
    }

    public HardwareQueue getHardwareQueue() {
        return hardwareQueue;
    }

    public Outtake getOuttake() {
        return outtake;
    }

    public Sensors getSensors() {
        return sensors;
    }
}
