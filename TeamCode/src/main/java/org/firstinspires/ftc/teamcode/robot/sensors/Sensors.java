package org.firstinspires.ftc.teamcode.robot.sensors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.robot.utils.wrappers.HardwareQueue;

public class Sensors implements Mechanism {
    private LynxModule controlHub;
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
    }

    @Override
    public void init(){
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    @Override
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

//        extendoEncoder = ((PriorityMotor) hardwareQueue.getDevice("intakeMotor")).motor[0].getCurrentPosition() * -1;
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
