package org.firstinspires.ftc.teamcode.config.utils.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PriorityServo {
    private Servo[] servo;
    private double position;

    public PriorityServo(Servo[] servo, HardwareMap hardwareMap) {
        this.servo = servo;
        this.position = 0;
    }
}
