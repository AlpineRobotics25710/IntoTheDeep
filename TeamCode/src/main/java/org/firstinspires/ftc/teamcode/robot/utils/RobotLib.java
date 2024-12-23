package org.firstinspires.ftc.teamcode.robot.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArmLib;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClawLib;

@Config
public class RobotLib {
    //hubs
    LynxModule controlHub;
    LynxModule expansionHub;
    double voltage;
    //intake
    public Servo intakeClaw;
    public Servo intakeSwivel;

    //intake arm
    public Servo iArmRight;
    public Servo iArmLeft;

    //intake wrist
    public Servo iWristLeft;
    public Servo iWristRight;

    //outtake

    //drivetrain
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public IntakeArmLib intakeArm; //we want to replace this with an overall intake class but its just an example rn
    public IntakeClawLib intakeEnd;
    public void init(HardwareMap map){
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //lmao we need to do this
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeSwivel = hardwareMap.get(Servo.class, "intakeSwivel");

        iArmLeft = hardwareMap.get(Servo.class, "iArmLeft");
        iArmRight = hardwareMap.get(Servo.class, "iArmRight");
        iWristLeft = hardwareMap.get(Servo.class, "iWristLeft");
        iWristRight = hardwareMap.get(Servo.class, "iWristRight");

        iArmRight.setDirection(Servo.Direction.REVERSE);
        iWristLeft.setDirection(Servo.Direction.REVERSE);

        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        //if we were to include our mechanisms they would go here
        intakeArm = new IntakeArmLib();
        intakeEnd = new IntakeClawLib();

        TelemetryUtil.setup();
        //follower outtake whatever goes here
    }
    private static RobotLib instance = new RobotLib();
    public boolean enabled;
    public static RobotLib getInstance(){
        if(instance == null){
            return new RobotLib();
        }
        instance.enabled = true;
        return instance;
        //im ngl i like have no idea what all of these checks are for but all i know is that like 100 people do this in their robot class and i get what its used for
    }

    public enum OpModeType{
        AUTO, TELEOP;
    }
}
