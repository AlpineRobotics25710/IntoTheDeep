package org.firstinspires.ftc.teamcode.config.sensors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.utils.wrappers.HardwareQueue;
import org.firstinspires.ftc.teamcode.config.utils.wrappers.PriorityMotor;

public class Sensors {
    private LynxModule controlHub;
    private LynxModule expansionHub;
    private HardwareMap hardwareMap;
    private HardwareQueue hardwareQueue;

    public Robot robot;
    private double extendoEncoder;
    private double extendoVel;
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
        //expansionHub.setBulkCachingMode ....

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

        extendoEncoder = ((PriorityMotor) hardwareQueue.getDevice("linkageMotor")).motor[0].getCurrentPosition() * -1; // this might have to be changed
//        extendoVel = ((PriorityMotor) hardwareQueue.getDevice("intakeMotor")).motor[0].getVelocity() * -1;
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



}