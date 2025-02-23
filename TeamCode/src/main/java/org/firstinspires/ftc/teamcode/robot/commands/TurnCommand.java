package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

//im so cool aren't i
public class TurnCommand extends CommandBase {
    private final Follower follower;
    private final double degrees;
    public TurnCommand(Follower follower, double degrees) {
        this.follower = follower;
        this.degrees = Math.toRadians(degrees);
    }
    @Override
    public void initialize() {
        follower.holdPoint(new Pose(follower.getPose().getX(), follower.getPose().getY(), degrees));
    }

    @Override
    public boolean isFinished() {
        TelemetryUtil.addData("Pos", follower.getPose().getHeading() + " " + degrees);
        return Math.abs(follower.getPose().getHeading() - degrees) <= 0.08;
    }
}