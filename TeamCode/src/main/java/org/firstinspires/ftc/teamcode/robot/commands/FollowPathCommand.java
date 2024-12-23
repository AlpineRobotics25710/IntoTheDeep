package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

//thank goodness ftc team watts-up is so cool that they integrated ftclib with pedro - rajveer
public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;

    public FollowPathCommand(Follower follower, PathChain path){
        this.follower = follower;
        this.path = path;
    }
    public FollowPathCommand(Follower follower, Path path){
        this(follower, new PathChain(path));
    }
    public FollowPathCommand setHoldEnd(boolean holdEnd){
        this.holdEnd = holdEnd;
        return this;
    }
    @Override
    public void initialize() {
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
