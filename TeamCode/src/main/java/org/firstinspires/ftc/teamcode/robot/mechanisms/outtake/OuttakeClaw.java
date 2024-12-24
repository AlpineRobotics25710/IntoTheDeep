package org.firstinspires.ftc.teamcode.robot.mechanisms.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;

@Config
public class OuttakeClaw extends Claw {
    private final HardwareMap hardwareMap;

    // TODO: NEED TO FIND THE CORRECT OPEN AND CLOSE POSITIONS FOR CLAW
    public static double CLAW_OPEN_POS = 0.0;
    public static double CLAW_CLOSED_POS = 0.0;

    public static double SWIVEL_TRANSFER_POS = 0.0;
    public static double SWIVEL_OUTTAKE_POS = 0.0;

    public OuttakeClaw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "outtakeClaw");
        swivelServo = hardwareMap.get(Servo.class, "outtakeClawSwivel");

        intake();
    }

    @Override
    public void ascend() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_OUTTAKE_POS);
    }

    @Override
    public void transfer() {
        setClawPosition(CLAW_OPEN_POS);
        setSwivelPosition(SWIVEL_TRANSFER_POS);
    }

    @Override
    public void intake() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_TRANSFER_POS);
    }

    @Override
    public void lowChamber() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_OUTTAKE_POS);
    }

    @Override
    public void highChamber() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_OUTTAKE_POS);
    }

    @Override
    public void lowBasket() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_OUTTAKE_POS);
    }

    @Override
    public void highBasket() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_OUTTAKE_POS);
    }
}
