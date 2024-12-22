package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

public class Sensors {
    private LynxModule controlHub;
    private LynxModule expansionHub;
    private HardwareMap hardwareMap;

    public Robot robot;

    private double voltage;
    private double voltageUpdateTime = 1000;
    long lastVoltageUpdatedTime = System.currentTimeMillis();

    public Sensors(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
    }

    public void init(){
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update(){
        updateControlHub();
    }

    public void updateControlHub(){
        long currTime = System.currentTimeMillis();
        if(currTime - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
    }

    public double getVoltage(){
        return voltage;
    }
}