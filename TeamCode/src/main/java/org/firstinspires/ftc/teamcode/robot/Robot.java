package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.lib.ExtendoSlidesLib;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.ExtendoSlides;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public class Robot {
    //hubs
    LynxModule controlHub;
    LynxModule expansionHub;
    double voltage;

    //intake

    //extendo
    public DcMotorEx extendoLeft;
    public DcMotorEx extendoRight;

    //end effector
    public Servo intakeClaw;
    public Servo intakeSwivel;

    //intake arm
    public Servo iArmRight;
    public Servo iArmLeft;

    //intake wrist
    public Servo iWristLeft;
    public Servo iWristRight;

    //outtake

    // outtake end effector
    public Servo outtakeClaw;
    public Servo outtakeSwivel;

    // outtake arm
    public Servo oArmRight;
    public Servo oArmLeft;

    // outtake wrist
    public Servo oWristLeft;
    public Servo oWristRight;

    // outtake slides
    public DcMotorEx slideLeft;
    public DcMotorEx slideRight;

    //drivetrain
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public IntakeArm intakeArm; //we want to replace this with an overall intake class but its just an example rn
    public IntakeClaw intakeEnd;
    public ExtendoSlides extendo;

    public OuttakeClaw outtakeEnd;

    private static Robot instance = new Robot();

    public void init(HardwareMap map){
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

        extendoLeft = map.get(DcMotorEx.class, "extendoLeft");
        extendoRight = map.get(DcMotorEx.class, "extendoRight");

        extendoLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // this needs to be changed with manual mode but ykw im lazy rn soooooooooo so skibidi sigma ohio rizz ;)
        extendoRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        //if teleop then we shouldn't reset encoders :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :)
        extendoLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendoLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        extendoRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendoRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        extendoLeft.setDirection(DcMotorEx.Direction.REVERSE);

        intakeClaw = map.get(Servo.class, "intakeClaw");
        intakeSwivel = map.get(Servo.class, "intakeSwivel");

        iArmLeft = map.get(Servo.class, "iArmLeft");
        iArmRight = map.get(Servo.class, "iArmRight");
        iWristLeft = map.get(Servo.class, "iWristLeft");
        iWristRight = map.get(Servo.class, "iWristRight");

        iArmRight.setDirection(Servo.Direction.REVERSE);
        iWristLeft.setDirection(Servo.Direction.REVERSE);

        controlHub = map.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = map.get(LynxModule.class, "Expansion Hub");

        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        voltage = map.voltageSensor.iterator().next().getVoltage();

        //if we were to include our mechanisms they would go here
        intakeArm = new IntakeArm();
        intakeEnd = new IntakeClaw();
        extendo = new ExtendoSlides();

        //follower outtake whatever goes here
        outtakeEnd = new OuttakeClaw();

        TelemetryUtil.setup();

    }

    public boolean enabled;

    public static Robot getInstance(){
        if(instance == null){
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
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

    public enum OpModeType{
        AUTO, TELEOP;
    }
}
