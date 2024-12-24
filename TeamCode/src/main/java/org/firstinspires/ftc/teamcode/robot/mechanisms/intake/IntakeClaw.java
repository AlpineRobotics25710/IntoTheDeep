package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;

@Config
public class IntakeClaw extends Claw {
    private final HardwareMap hardwareMap;

    // TODO: NEED TO FIND RIGHT POSITIONS FOR INTAKE CLAW
    public static double CLAW_OPEN_POS = 0.3; // DONE
    public static double CLAW_CLOSED_POS = 0.55; // DONE
    public static double SWIVEL_INTAKE_POS = 0.0;
    public static double SWIVEL_TRANSFER_POS = 0.0;
    public static double SWIVEL_ASCENT_POS = 0.0;

    public IntakeClaw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void setClawState(ClawState state) {
        clawState = state;
        switch (clawState) {
            case OPEN:
                setClawPosition(CLAW_OPEN_POS);
                break;

            case CLOSED:
                setClawPosition(CLAW_CLOSED_POS);
                break;
        }
    }

    public void setSwivelState(SwivelState state) {
        swivelState = state;
        switch (swivelState) {
            case INTAKE:
                setSwivelPosition(SWIVEL_INTAKE_POS);
                break;

            case TRANSFER:
                setSwivelPosition(SWIVEL_TRANSFER_POS);
                break;
        }
    }

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "intakeClaw");
        swivelServo = hardwareMap.get(Servo.class, "intakeSwivel");

        intake();
    }

    @Override
    public void ascend() {
        transfer();
    }

    @Override
    public void transfer() {
        setClawState(ClawState.CLOSED);
        setSwivelState(SwivelState.TRANSFER);
    }

    @Override
    public void intake() {
        setClawState(ClawState.OPEN);
        setSwivelState(SwivelState.INTAKE);
    }

    @Override
    public void lowChamber() {
        intake();
    }

    @Override
    public void highChamber() {
        intake();
    }

    @Override
    public void lowBasket() {
        intake();
    }

    @Override
    public void highBasket() {
        intake();
    }
}
