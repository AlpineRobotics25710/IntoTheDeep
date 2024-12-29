package org.firstinspires.ftc.teamcode.robot.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.robot.RobotLib;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
public class ExtendoSlides extends SubsystemBase {
    private static RobotLib robot = RobotLib.getInstance();
    public static double BASE_POS = 0.0;
    public static double TRANSFER_POS = 0.0;
    public static double INTAKE_POS = 0.0;

    public static double Kp;
    public static double Ki;
    public static double Kd;
    public static double Kf;

    public double slidesPower = 0.0;

    public ExtendoSlides() {
       // super(Kp, Ki, Kd, Kf);
        //this.leftMotor = robot.extendoLeft;
      //  this.rightMotor = robot.extendoRight;
    }

    public void setSlidesPower(double slidesPower) {
        this.slidesPower = slidesPower;
    }

    public void moveSlides() {
        TelemetryUtil.packet.put("slidfes power", slidesPower);
        robot.extendoLeft.setPower(slidesPower);
        robot.extendoRight.setPower(slidesPower);
        TelemetryUtil.packet.put("slides power from actual method", robot.extendoLeft.getPower());
    }

    @Override
    public void periodic() {
        moveSlides();
    }

    //@Override
    public void init() {
        //setTargetPosition(BASE_POS);
    }
}
