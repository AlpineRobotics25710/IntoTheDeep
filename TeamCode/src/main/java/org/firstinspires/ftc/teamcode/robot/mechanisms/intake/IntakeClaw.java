package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.MechanismState;

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

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "intakeClaw");
        swivelServo = hardwareMap.get(Servo.class, "intakeSwivel");

        intake();
    }

    @Override
    public void ascend() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_ASCENT_POS);
        clawState = MechanismState.ASCENT;
    }

    @Override
    public void transfer() {
        setClawPosition(CLAW_OPEN_POS);
        setSwivelPosition(SWIVEL_TRANSFER_POS);
        clawState = MechanismState.TRANSFER;
    }

    @Override
    public void intake() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_TRANSFER_POS);
        clawState = MechanismState.INTAKE;
    }

    @Override
    public void lowChamber() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_INTAKE_POS);
        clawState = MechanismState.LOW_CHAMBER;
    }

    @Override
    public void highChamber() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_INTAKE_POS);
        clawState = MechanismState.HIGH_CHAMBER;
    }

    @Override
    public void lowBasket() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_INTAKE_POS);
        clawState = MechanismState.LOW_BASKET;
    }

    @Override
    public void highBasket() {
        setClawPosition(CLAW_CLOSED_POS);
        setSwivelPosition(SWIVEL_INTAKE_POS);
        clawState = MechanismState.HIGH_BASKET;
    }
}
