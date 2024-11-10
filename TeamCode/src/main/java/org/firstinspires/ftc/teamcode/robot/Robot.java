package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo2;
import org.firstinspires.ftc.teamcode.robot.utils.Constants;
import org.firstinspires.ftc.teamcode.robot.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.robot.sensors.Sensors;

public class Robot {
    public HardwareQueue hardwareQueue;
    public Sensors sensors;
    public Extendo2 extendo;
    public Robot(HardwareMap hardwareMap){
        hardwareQueue = new HardwareQueue();
        sensors = new Sensors(hardwareMap, hardwareQueue, this);
        extendo = new Extendo2(hardwareMap, hardwareQueue, sensors);
    }

    public void update(){
        Constants.START_LOOP();
        hardwareQueue.update();
        sensors.update();
        extendo.update();
    }

}
