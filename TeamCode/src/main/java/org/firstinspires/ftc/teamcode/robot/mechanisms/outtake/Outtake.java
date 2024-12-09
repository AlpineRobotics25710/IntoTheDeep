package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;

public class Outtake implements Mechanism {
    private HardwareMap hardwareMap;
    private Claw claw;
    private Arm arm;
    private Slides slides;

    public Outtake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        slides = new Slides(hardwareMap);
    }

    public void transfer() {
    }

    @Override
    public void init() {
        claw.init();
        arm.init();
        slides.init();
    }

    @Override
    public void update() {
        claw.update();
        arm.update();
        slides.update();
        Mechanism.super.update();
    }

    public Claw getClaw() {
        return claw;
    }

    public Arm getArm() {
        return arm;
    }

    public Slides getSlides() {
        return slides;
    }
}
