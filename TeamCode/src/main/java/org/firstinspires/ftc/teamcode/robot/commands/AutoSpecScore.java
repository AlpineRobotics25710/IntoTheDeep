package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.WaypointConstants;
import org.firstinspires.ftc.teamcode.robot.utils.WaypointGenerator;


public class AutoSpecScore extends SequentialCommandGroup {
    public AutoSpecScore(Robot robot) {
        super(
             new ParallelCommandGroup(
                     new FollowPathCommand(robot.follower, WaypointGenerator.getSubmersiblePath(robot.follower)),
                     new SequentialCommandGroup(
                             new WaitCommand(100),
                             new OuttakeIntermediateCommand(robot)
                     )
             ),
             new HighChamberCommand(robot)
        );
    }
}
