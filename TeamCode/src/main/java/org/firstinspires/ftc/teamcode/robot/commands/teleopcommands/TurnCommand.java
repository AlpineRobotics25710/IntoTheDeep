package org.firstinspires.ftc.teamcode.robot.commands.teleopcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.robot.commands.FollowPathCommand;
//im so cool aren't i
public class TurnCommand extends CommandBase {
    private final Follower follower;
    private final double degrees;
    public TurnCommand(Follower follower, double degrees) {
        this.follower = follower;
        this.degrees = degrees;
    }
    @Override
    public void initialize() {
        follower.holdPoint(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(degrees)));
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}