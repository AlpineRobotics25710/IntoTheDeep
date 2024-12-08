package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.utils.Globals;
import org.firstinspires.ftc.teamcode.config.subsystem.intake.Extendo;
import org.firstinspires.ftc.teamcode.config.utils.wrappers.HardwareQueue;
import org.firstinspires.ftc.teamcode.config.sensors.Sensors;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class Robot {
    public HardwareQueue hardwareQueue;
    public Sensors sensors;
    public Extendo extendo;
    public Follower follower;
    public Robot(HardwareMap hardwareMap){
        hardwareQueue = new HardwareQueue();
        sensors = new Sensors(hardwareMap, hardwareQueue, this);
        follower = new Follower(hardwareMap);
        extendo = new Extendo(hardwareMap, hardwareQueue, sensors);
    }

    public void update(){
        Globals.START_LOOP();
//        hardwareQueue.update();
        follower.update(); //add follower to hardwareQueue
        sensors.update();
        extendo.update();
    }

}