package org.firstinspires.ftc.teamcode.config.sensors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.config.utils.priority.PriorityMotor;

public class Sensors {
    private LynxModule controlHub;
    private LynxModule expansionHub;
    private HardwareMap hardwareMap;
    private HardwareQueue hardwareQueue;

    public Robot robot;
    private double extendoEncoder;
    private double extendoVel;

    private double slidesEncoder;
    private double slidesVel;
    private double voltage;
    public Sensors(HardwareMap hardwareMap, HardwareQueue queue, Robot robot){
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = queue;
        this.robot = robot;

        initSensors(hardwareMap);
    }

    public void initSensors(HardwareMap hardwareMap){
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //NEED TO ADD EXPANSION HUB HERE
        //expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
        //expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }
    public void update(){
        updateControlHub();
    }

    private double voltageUpdateTime = 1000;
    long lastVoltageUpdatedTime = System.currentTimeMillis();
    public void updateControlHub(){
        long currTime = System.currentTimeMillis();
        if(currTime - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

        extendoEncoder = ((PriorityMotor) hardwareQueue.getDevice("linkageMotor")).motor[0].getCurrentPosition(); // this might have to be changed to * -1
//        extendoVel = ((PriorityMotor) hardwareQueue.getDevice("linkageMotor")).motor[0].getVelocity() * -1;
        slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("slidesMotor")).motor[0].getCurrentPosition(); //this might have to be changed to * -1
    }

    public double getVoltage(){
        return voltage;
    }

    public double getExtendoPos(){
        return extendoEncoder;
    }

    public double getExtendoVel(){
        return extendoVel;
    }


    public double getSlidesPos() {
        return slidesEncoder;
    }
}