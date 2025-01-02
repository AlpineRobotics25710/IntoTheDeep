package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;

@Config
public class SetIntake extends CommandBase {
    public static final double armWaitTime = 500.0; //500 ms for now
    private final IntakeEnd intakeEnd;
    private final IntakeArm intakeArm;
    private final Extendo extendo;
    private final IntakeArm.IntakeArmState armState;
    private final IntakeEnd.ActiveState activeState;
    private final double target;
    ElapsedTime timer;
    private boolean waitForArm;
    public SetIntake(Robot robot, double target, IntakeArm.IntakeArmState armState, IntakeEnd.ActiveState activeState, boolean waitForArm){
        intakeEnd = robot.intakeEnd;
        intakeArm = robot.intakeArm;
        extendo = robot.extendo;
        this.armState = armState;
        this.activeState = activeState;
        this.target = target;
        this.waitForArm = waitForArm;
        if(armState.equals(IntakeArm.currentState)){
            waitForArm = false;
        }
        timer = new ElapsedTime();
        addRequirements(robot.extendo, robot.intakeArm, robot.intakeEnd);
    }

    @Override
    public void initialize(){
        intakeArm.setState(armState);
        if(activeState.equals(IntakeEnd.ActiveState.FORWARD)){
            intakeEnd.setState(activeState);
        }
        extendo.setTargetPosition(target);
        timer.reset();
    }
    @Override
    public void execute(){
        if(waitForArm){
            if(timer.milliseconds() > armWaitTime && activeState.equals(IntakeEnd.ActiveState.REVERSED)){
                intakeEnd.setState(activeState);
                waitForArm = false;
            }
        }
    }
    @Override
    public boolean isFinished() {
        return extendo.extendoReached && timer.milliseconds() > armWaitTime;
    }
}
