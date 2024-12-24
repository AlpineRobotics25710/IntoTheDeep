package org.firstinspires.ftc.teamcode.robot.lib;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public class RobotLib {
    //hubs
    LynxModule controlHub;
    LynxModule expansionHub;
    double voltage;
    //intake

    //extendo
    public DcMotor extendoLeft;
    public DcMotor extendoRight;
    //end
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
    public ExtendoSlidesLib extendo;
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

        extendoLeft = map.get(DcMotor.class, "extendoLeft");
        extendoRight = map.get(DcMotor.class, "extendoRight");

        extendoLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // this needs to be changed with manual mode but ykw im lazy rn soooooooooo so skibidi sigma ohio rizz ;)
        extendoRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //if teleop then we shouldn't reset encoders :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :) :)
        extendoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendoLeft.setDirection(DcMotor.Direction.REVERSE);

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
        intakeArm = new IntakeArmLib();
        intakeEnd = new IntakeClawLib();
        extendo = new ExtendoSlidesLib();



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
