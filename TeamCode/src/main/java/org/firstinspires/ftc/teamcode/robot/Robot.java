package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo2;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.robot.utils.Constants;
import org.firstinspires.ftc.teamcode.robot.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.robot.sensors.Sensors;

public class Robot {
    private final HardwareQueue hardwareQueue;
    private final Sensors sensors;
    private final Outtake outtake;
    private Extendo2 extendo;

    public Robot(HardwareMap hardwareMap) {
        hardwareQueue = new HardwareQueue();
        sensors = new Sensors(hardwareMap, hardwareQueue, this);
        outtake = new Outtake(hardwareMap);
        //extendo = new Extendo2(hardwareMap, hardwareQueue, sensors);
    }

    public void init() {
        outtake.init();
        //extendo.init();
    }

    public void update() {
        Constants.START_LOOP();
        hardwareQueue.update();
        sensors.update();
        outtake.update();
        //extendo.update();
    }

    public HardwareQueue getHardwareQueue() {
        return hardwareQueue;
    }
}
