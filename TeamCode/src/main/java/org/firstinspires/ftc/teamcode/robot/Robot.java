package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robot.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.robot.sensors.Sensors;

public class Robot {
    private final HardwareQueue hardwareQueue;
    private final Sensors sensors;
    private final Outtake outtake;

    public Robot(HardwareMap hardwareMap) {
        hardwareQueue = new HardwareQueue();
        sensors = new Sensors(hardwareMap, hardwareQueue, this);
        outtake = new Outtake(hardwareMap);

        assert RobotConstants.alliance != null;
        assert RobotConstants.mode != null;
    }

    public void init() {
        outtake.init();
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
}
