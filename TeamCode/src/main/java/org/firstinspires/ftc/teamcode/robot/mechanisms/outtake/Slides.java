package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

public class Slides implements Mechanism {
    private HardwareMap hardwareMap;

    public Slides(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {
        Mechanism.super.update();
    }
}
