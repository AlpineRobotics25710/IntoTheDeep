package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.commands.AutonInitializeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TeleOpInitializeCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.robot.utils.WaypointConstants;

import java.util.List;

@Config
public class Robot {
    //hubs
    public List<LynxModule> allHubs;
    public double voltage;

    // Intake
    // Extendo motors
    public DcMotor extendoRight;

    // Intake end effectors servos
    public CRServo activeIntake;

    // Intake arm servos
    public Servo iArmRight;
    public Servo iArmLeft;

    // Intake wrist servos
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
    public Servo oWrist;

    public IntakeArm intakeArm;
    public IntakeEnd intakeEnd;
    public Extendo extendo;
    public OuttakeClaw outtakeClaw;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;

    public Follower follower;

    public Robot(HardwareMap hardwareMap, boolean isAuto) {
        // Pedro
        Constants.setConstants(FConstants.class, LConstants.class);

        // Configuration of all motors and servos
        extendoRight = hardwareMap.get(DcMotor.class, "extendoRight");

        iArmLeft = hardwareMap.get(Servo.class, "iArmLeft");
        iArmRight = hardwareMap.get(Servo.class, "iArmRight");
        iWristRight = hardwareMap.get(Servo.class, "iWristRight");
        activeIntake = hardwareMap.get(CRServo.class, "activeIntake");

        outtakeSlideLeft = hardwareMap.get(DcMotor.class, "slidesLeft");
        outtakeSlideRight = hardwareMap.get(DcMotor.class, "slidesRight");

        oArmLeft = hardwareMap.get(Servo.class, "oArmLeft");
        oArmRight = hardwareMap.get(Servo.class, "oArmRight");
        oWrist = hardwareMap.get(Servo.class, "oWrist");
        outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");
        outtakeSwivelServo = hardwareMap.get(Servo.class, "outtakeSwivelServo");

        allHubs = hardwareMap.getAll(LynxModule.class);

        // Set directions of all motors and servos
        extendoRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeSlideLeft.setDirection(DcMotor.Direction.FORWARD);
        outtakeSlideRight.setDirection(DcMotor.Direction.REVERSE);
        iArmRight.setDirection(Servo.Direction.REVERSE);
        oArmRight.setDirection(Servo.Direction.FORWARD);
        oArmLeft.setDirection(Servo.Direction.REVERSE);
        oWrist.setDirection(Servo.Direction.FORWARD);
        //outtakeSwivelServo.setDirection(Servo.Direction.REVERSE);

        // Resetting encoders
        if (isAuto) {
            extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

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
        intakeArm = new IntakeArm(iArmRight, iArmLeft, iWristRight);
        intakeEnd = new IntakeEnd(activeIntake);
        extendo = new Extendo(extendoRight, false);
        outtakeClaw = new OuttakeClaw(outtakeClawServo, outtakeSwivelServo);
        outtakeSlides = new OuttakeSlides(outtakeSlideLeft, outtakeSlideRight, false);
        outtakeArm = new OuttakeArm(oArmRight, oArmLeft, oWrist);
        follower = new Follower(hardwareMap);
        WaypointConstants.follower = follower;

        // Register all subsystems
        CommandScheduler.getInstance().registerSubsystem(intakeArm, intakeEnd, extendo, outtakeClaw, outtakeSlides, outtakeArm);

        if (isAuto) {
            new AutonInitializeCommand(this).schedule();
        } else {
            new TeleOpInitializeCommand(this).schedule();
        }
        //CommandScheduler.getInstance().setDefaultCommand(intakeEnd, new IntakeEndCommand(this, IntakeEnd.ActiveState.OFF));
    }

    public void resetExtendoEncoders() {
        extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop() {
        CommandScheduler.getInstance().run();
        follower.update();
        TelemetryUtil.addData("intake arm pos", intakeArm.getArmPosition());
        TelemetryUtil.addData("intake wrist pos", intakeArm.getWristPosition());
        TelemetryUtil.addData("Intake state", intakeArm.currentState);
        TelemetryUtil.addData("Current Arm State", intakeArm.currentState);
        TelemetryUtil.addData("outtake arm pos", outtakeArm.getArmPosition());
        TelemetryUtil.addData("outtake wrist pos", oWrist.getPosition());
        TelemetryUtil.addData("Current outtake state", outtakeArm.getCurrentState());
        TelemetryUtil.addData("Outtake slides pos", outtakeSlides.getCurrentPosition());
        TelemetryUtil.addData("outtake slides target pos", outtakeSlides.getTargetPosition());

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    private void clearHubCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    private void reset() {
        CommandScheduler.getInstance().reset();
    }

    public void end() {
        reset();
        clearHubCache();
    }

    public double getVoltage() {
        return voltage;
    }
}
