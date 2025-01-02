package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;
@Config
public class SetOuttake extends CommandBase {
    private static final double armWaitTime = 500.0; // 500ms wait for arm movement
    private static final double clawWaitTime = 200.0; // 200ms wait for claw movement

    private final OuttakeArm outtakeArm;
    private final OuttakeClaw outtakeClaw;
    private final OuttakeSlides slides;
    private final OuttakeArm.OuttakeArmState armState;
    private final OuttakeClaw.OuttakeClawState clawState;
    private final OuttakeClaw.OuttakeSwivelState swivelState;
    private final double target;
    private ElapsedTime timer;
    private boolean waitForSlides;
    private boolean armAndSwivelMoved;

    // Constructor when claw movement is required
    public SetOuttake(Robot robot, double target, OuttakeArm.OuttakeArmState armState,
                      OuttakeClaw.OuttakeClawState clawState, OuttakeClaw.OuttakeSwivelState swivelState) {
        this.outtakeArm = robot.outtakeArm;
        this.outtakeClaw = robot.outtakeClaw;
        this.slides = robot.outtakeSlides;
        this.target = target;
        this.armState = armState;
        this.clawState = clawState;
        this.swivelState = swivelState;
        this.waitForSlides = !slides.extendoReached;
        this.armAndSwivelMoved = false;

        this.timer = new ElapsedTime();
        addRequirements(outtakeArm, outtakeClaw, slides);
    }

    // Constructor when claw movement is not required
    public SetOuttake(Robot robot, double target, OuttakeArm.OuttakeArmState armState,
                      OuttakeClaw.OuttakeSwivelState swivelState) {
        this(robot, target, armState, null, swivelState);
    }

    @Override
    public void initialize() {
        if (waitForSlides) {
            slides.setTargetPosition(target); // Start moving slides if necessary
        } else {
            moveArmAndSwivel(); // If slides are already in position, move arm and swivel
            timer.reset();
        }
    }

    @Override
    public void execute() {
        if (waitForSlides && slides.extendoReached) {
            waitForSlides = false;
            moveArmAndSwivel(); // Once slides reach target, move arm and swivel
            timer.reset();
        }

        if (!waitForSlides && !armAndSwivelMoved && timer.milliseconds() > armWaitTime) {
            armAndSwivelMoved = true;
            moveClawIfNeeded(); // Move the claw after arm/swivel if required
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return !waitForSlides && armAndSwivelMoved && (clawState == null || timer.milliseconds() > clawWaitTime);
    }

    private void moveArmAndSwivel() {
        outtakeArm.setState(armState);
        outtakeClaw.setSwivelState(swivelState);
    }

    private void moveClawIfNeeded() {
        if (clawState != null) {
            outtakeClaw.setClawState(clawState);
        }
    }
}
