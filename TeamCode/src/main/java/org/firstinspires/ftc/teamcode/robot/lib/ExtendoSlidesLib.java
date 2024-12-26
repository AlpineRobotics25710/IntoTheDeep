package org.firstinspires.ftc.teamcode.robot.lib;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class ExtendoSlidesLib extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public double target;
    public final double MAX_EXTENDO_EXTENSION = 0.0;
    public boolean extendoReached;
    public boolean extendoRetracted;

    private static final PIDFController extendoPIDF = new PIDFController(0.01,0,0.000001, 0); //example values for now

    public void init() {
        setExtendoTarget(0);
        extendoPIDF.setTolerance(15);
    }

    public void autoUpdateExtendo() {
        double extendoPower = extendoPIDF.calculate(robot.extendoRight.getCurrentPosition(), this.target);
        extendoReached = (extendoPIDF.atSetPoint() && target > 0) || (robot.extendoRight.getCurrentPosition() <= 3 && target == 0); // 3 is example number
        extendoRetracted = (target <= 0) && extendoReached;

        // Just make sure it gets to fully retracted if target is 0
        if (target == 0) {
            extendoPower -= 0.2;
        } else {
            extendoPower += 0.2;
        }

        if (extendoReached) {
            robot.extendoLeft.setPower(0);
            robot.extendoRight.setPower(0);
        } else {
            robot.extendoLeft.setPower(extendoPower);
            robot.extendoRight.setPower(extendoPower);
        }
    }

    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
    }

    @Override
    public void periodic() {
        autoUpdateExtendo();
    }
}