package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.ExtendoSlides;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public class Robot {
    public static OpModeType opModeType;
    //hubs
    public LynxModule controlHub;

    //intake
    public LynxModule expansionHub;
    public double voltage;
    //extendo
    public DcMotor extendoLeft;
    public DcMotor extendoRight;
    //end effector
    public Servo intakeClawServo;
    public Servo intakeSwivelServo;
    //intake arm
    public Servo iArmRight;
    public Servo iArmLeft;

    //outtake
    //intake wrist
    public Servo iWristLeft;
    public Servo iWristRight;
    // outtake end effector
    public Servo outtakeClawServo;
    public Servo outtakeSwivelServo;
    // outtake arm
    public Servo oArmRight;
    public Servo oArmLeft;
    // outtake wrist
    public Servo oWristLeft;
    public Servo oWristRight;
    // outtake slides
    public DcMotor outtakeSlideLeft;
    public DcMotor outtakeSlideRight;
    //drivetrain
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public IntakeArm intakeArm; //we want to replace this with an overall intake class but its just an example rn
    public IntakeClaw intakeClaw;
    public ExtendoSlides extendo;
    public OuttakeClaw outtakeClaw;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    //public HardwareMap map;
    public boolean enabled;
    long lastVoltageUpdatedTime = System.currentTimeMillis();

    public void init(HardwareMap map) {
        //this.map = map;

        // Drivetrain motors (but we should be using PedroDrivetrain)
        frontLeftMotor = map.get(DcMotor.class, ("frontLeftMotor"));
        backLeftMotor = map.get(DcMotor.class, ("backLeftMotor"));
        frontRightMotor = map.get(DcMotor.class, ("frontRightMotor"));
        backRightMotor = map.get(DcMotor.class, ("backRightMotor"));

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //lmao we need to do this
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Extendo
        extendoLeft = map.get(DcMotor.class, "extendoLeft");
        extendoRight = map.get(DcMotor.class, "extendoRight");

        extendoLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // this needs to be changed with manual mode but ykw im lazy rn soooooooooo so skibidi sigma ohio rizz ;)
        extendoRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TelemetryUtil.packet.put("extendo", "initialized");
        // if teleop then we shouldn't reset encoders
        extendoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendoLeft.setDirection(DcMotor.Direction.REVERSE);
        extendoRight.setDirection(DcMotor.Direction.FORWARD);

        // Outtake slides
        /*outtakeSlideLeft = map.get(DcMotor.class, "outtakeLeft");
        outtakeSlideRight = map.get(DcMotor.class, "outtakeRight");

        outtakeSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // if teleop then we shouldn't reset encoders
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeSlideRight.setDirection(DcMotor.Direction.FORWARD);*/

        // Opmode type dependant functions
        //assert opModeType != null;

        if (opModeType != OpModeType.TELEOP) {
            extendoRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            extendoLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            /*outtakeSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);*/
        }

        // Intake
       /* intakeClawServo = map.get(Servo.class, "intakeClawServo");
        intakeSwivelServo = map.get(Servo.class, "intakeSwivelServo");

        iArmLeft = map.get(Servo.class, "iArmLeft");
        iArmRight = map.get(Servo.class, "iArmRight");
        iWristLeft = map.get(Servo.class, "iWristLeft");
        iWristRight = map.get(Servo.class, "iWristRight");

        iArmRight.setDirection(Servo.Direction.REVERSE);
        iWristLeft.setDirection(Servo.Direction.REVERSE);*/

        // Outtake
       /* outtakeClawServo = map.get(Servo.class, "outtakeClawServo");
        outtakeSwivelServo = map.get(Servo.class, "outtakeSwivelServo");

        oArmLeft = map.get(Servo.class, "oArmLeft");
        oArmRight = map.get(Servo.class, "oArmRight");
        oWristLeft = map.get(Servo.class, "oWristLeft");
        oWristRight = map.get(Servo.class, "oWristRight");

        oArmRight.setDirection(Servo.Direction.REVERSE);
        oWristLeft.setDirection(Servo.Direction.REVERSE);*/

        // Hubs
        controlHub = map.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);

        expansionHub = map.get(LynxModule.class, "Expansion Hub");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);

        voltage = map.voltageSensor.iterator().next().getVoltage();

        // Initialize all mechanisms
//        intakeArm = new IntakeArm();
//        intakeClaw = new IntakeClaw();
        extendo = new ExtendoSlides();

        // Outtake
//        outtakeClaw = new OuttakeClaw();
//        outtakeSlides = new OuttakeSlides();
//        outtakeArm = new OuttakeArm();

        // Follower maybe?

//        intakeArm.init();
//        intakeClaw.init();
       // extendo.init();
//        outtakeClaw.init();
//        outtakeSlides.init();
//        outtakeArm.init();

        TelemetryUtil.setup();
    }

    private static Robot instance = new Robot();

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    public void updateControlHub() {
        long currTime = System.currentTimeMillis();
        double voltageUpdateTime = 1000;
        if (currTime - lastVoltageUpdatedTime > voltageUpdateTime) {
            //voltage = map.voltageSensor.iterator().next().getVoltage();
        }
    }

    public void ascend() {

    }

    public void transfer() {

    }

    public void intake() {

    }

    public void lowChamber() {

    }

    public void highChamber() {
        
    }

    public void lowBasket() {

    }

    public void highBasket() {

    }

    public enum OpModeType {
        AUTO, TELEOP
    }
}
