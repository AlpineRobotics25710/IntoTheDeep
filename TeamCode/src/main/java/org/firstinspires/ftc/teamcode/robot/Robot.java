package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

import java.util.List;

@Config
public class Robot {
    //hubs
    public List<LynxModule> allHubs;
    public double voltage;

    // Intake
    // Extendo motors
    public DcMotor extendoLeft;
    public DcMotor extendoRight;

    // Intake end effectors servos
    public CRServo activeIntake;
    public Servo intakeSwivelServo;

    // Intake arm servos
    public Servo iArmRight;
    public Servo iArmLeft;

    // Intake wrist servos
    public Servo iWristLeft;
    public Servo iWristRight;

    // Outtake
    // outtake slides
    public DcMotor outtakeSlideLeft;
    public DcMotor outtakeSlideRight;

    // outtake end effector
    public Servo outtakeClawServo;
    public Servo outtakeSwivelServo;

    // Outtake arm servos
    public Servo oArmRight;
    public Servo oArmLeft;

    // Outtake wrist servos
    public Servo oWristLeft;
    public Servo oWristRight;

    public IntakeArm intakeArm;
    public IntakeEnd intakeEnd;
    public Extendo extendo;
    public OuttakeClaw outtakeClaw;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;

    public Robot(HardwareMap hardwareMap, boolean isAuto, boolean manualMode) {
        // Configuration of all motors and servos
        extendoLeft = hardwareMap.get(DcMotor.class, "extendoLeft");
        extendoRight = hardwareMap.get(DcMotor.class, "extendoRight");
        activeIntake = hardwareMap.get(CRServo.class, "activeIntake");

        iArmLeft = hardwareMap.get(Servo.class, "iArmLeft");
        iArmRight = hardwareMap.get(Servo.class, "iArmRight");
        iWristLeft = hardwareMap.get(Servo.class, "iWristLeft");
        iWristRight = hardwareMap.get(Servo.class, "iWristRight");

        outtakeSlideLeft = hardwareMap.get(DcMotor.class, "outtakeLeft");
        outtakeSlideRight = hardwareMap.get(DcMotor.class, "outtakeRight");
        outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");
        outtakeSwivelServo = hardwareMap.get(Servo.class, "outtakeSwivelServo");

        oArmLeft = hardwareMap.get(Servo.class, "oArmLeft");
        oArmRight = hardwareMap.get(Servo.class, "oArmRight");
        oWristLeft = hardwareMap.get(Servo.class, "oWristLeft");
        oWristRight = hardwareMap.get(Servo.class, "oWristRight");

        allHubs = hardwareMap.getAll(LynxModule.class);

        // Set directions of all motors and servos
        extendoLeft.setDirection(DcMotor.Direction.REVERSE);
        extendoRight.setDirection(DcMotor.Direction.FORWARD);
        outtakeSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeSlideRight.setDirection(DcMotor.Direction.FORWARD);
        iArmRight.setDirection(Servo.Direction.REVERSE);
        iWristLeft.setDirection(Servo.Direction.REVERSE);
        oArmRight.setDirection(Servo.Direction.REVERSE);
        oWristLeft.setDirection(Servo.Direction.REVERSE);

        // Resetting encoders
        if (isAuto) {
            extendoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        extendoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Bulk caching mode of hubs
        // Bulk reading enabled!
        // AUTO mode will bulk read by default and will redo and clear cache once the exact same read is done again
        // MANUAL mode will bulk read once per loop but needs to be manually cleared
        // Also in opModes only clear ControlHub cache as it is a hardware write
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        //remember to clear cache at the end of opmodes :0

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Initialize all mechanisms
        intakeArm = new IntakeArm(iArmRight, iArmLeft, iWristRight, iWristLeft);
        intakeEnd = new IntakeEnd(activeIntake);
        extendo = new Extendo(extendoLeft, extendoRight, manualMode);
        outtakeClaw = new OuttakeClaw(outtakeClawServo, outtakeSwivelServo);
        outtakeSlides = new OuttakeSlides(outtakeSlideLeft, outtakeSlideRight, manualMode);
        outtakeArm = new OuttakeArm(oArmRight, oArmLeft, oWristRight, oWristLeft);

        // Register all subsystems
        CommandScheduler.getInstance().registerSubsystem(
                intakeArm,
                intakeEnd,
                extendo,
                outtakeClaw,
                outtakeSlides,
                outtakeArm
        );
    }

    public void loop() {
        CommandScheduler.getInstance().run();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void clearHubCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    public double getVoltage() {
        return voltage;
    }
}
