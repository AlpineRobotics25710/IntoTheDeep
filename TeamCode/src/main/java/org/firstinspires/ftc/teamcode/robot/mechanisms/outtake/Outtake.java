package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

public class Outtake implements Mechanism {
    private HardwareMap hardwareMap;
    private Claw claw;
    private Arm arm;

    public Outtake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
    }

    public void transfer() {
    }

    @Override
    public void init() {
        claw.init();
        arm.init();
    }

    @Override
    public void update() {
        claw.update();
        arm.update();
        Mechanism.super.update();
    }
}
