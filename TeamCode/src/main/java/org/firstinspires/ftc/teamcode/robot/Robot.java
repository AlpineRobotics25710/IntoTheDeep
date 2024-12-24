package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.robot.mechanisms.MechanismState;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Sensors;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.ExtendoSlides;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;
import org.firstinspires.ftc.teamcode.robot.utils.Globals;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

public class Robot implements Mechanism {
    public HardwareMap hardwareMap;
    public OuttakeArm outtakeArm;
    public OuttakeClaw outtakeClaw;
    public IntakeArm intakeArm;
    public IntakeClaw intakeClaw;
    public OuttakeSlides outtakeSlides;
    public ExtendoSlides extendo;
    public Sensors sensors;
    public MechanismState robotState;
    public boolean enabled;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    @Override
    public void init() {
        TelemetryUtil.setup();
        sensors = new Sensors(hardwareMap, this);
        sensors.init();

        // Initialize outtake
        outtakeArm = new OuttakeArm(hardwareMap);
        outtakeClaw = new OuttakeClaw(hardwareMap);
        outtakeSlides = new OuttakeSlides(hardwareMap);
        outtakeArm.init();
        outtakeClaw.init();
        outtakeSlides.init();

        // Initialize intake
        intakeArm = new IntakeArm(hardwareMap);
        intakeClaw = new IntakeClaw(hardwareMap);
        extendo = new ExtendoSlides(hardwareMap);
        intakeArm.init();
        intakeClaw.init();
        extendo.init();
    }

    @Override
    public void update() {
        Globals.START_LOOP();
        outtakeSlides.update();
        extendo.update();
        TelemetryUtil.packet.put("Loop time (seconds)", Globals.GET_LOOP_TIMES());
        TelemetryUtil.sendTelemetry();
    }

    public void ascend() {
        outtakeSlides.ascend();
        outtakeArm.setState(MechanismState.ASCEND);
        robotState = MechanismState.ASCEND;
    }

    public void transfer() {
        outtakeArm.transfer();
        outtakeClaw.transfer();
        outtakeSlides.transfer();
        intakeArm.transfer();
        intakeClaw.transfer();
        extendo.transfer();
        robotState = MechanismState.TRANSFER;
    }

    public void intake() {
        outtakeArm.intake();
        outtakeClaw.intake();
        outtakeSlides.intake();
        intakeArm.intake();
        intakeClaw.intake();
        extendo.intake();
        robotState = MechanismState.INTAKE;
    }

    public void lowChamber() {
        outtakeArm.lowChamber();
        outtakeClaw.lowChamber();
        outtakeSlides.lowChamber();
        intakeArm.lowChamber();
        intakeClaw.lowChamber();
        extendo.lowChamber();
        robotState = MechanismState.LOW_CHAMBER;
    }

    public void highChamber() {
        outtakeArm.highChamber();
        outtakeClaw.highChamber();
        outtakeSlides.highChamber();
        intakeArm.highChamber();
        intakeClaw.highChamber();
        extendo.highChamber();
        robotState = MechanismState.HIGH_CHAMBER;
    }

    public void lowBasket() {
        outtakeArm.lowBasket();
        outtakeClaw.lowBasket();
        outtakeSlides.lowBasket();
        intakeArm.lowBasket();
        intakeClaw.lowBasket();
        extendo.lowBasket();
        robotState = MechanismState.LOW_BASKET;
    }

    public void highBasket() {
        outtakeArm.highBasket();
        outtakeClaw.highBasket();
        outtakeSlides.highBasket();
        intakeArm.highBasket();
        intakeClaw.highBasket();
        extendo.highBasket();
        robotState = MechanismState.HIGH_BASKET;
    }
}
