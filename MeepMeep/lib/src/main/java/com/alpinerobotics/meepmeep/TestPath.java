package com.alpinerobotics.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class TestPath {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(0.15, -30, Math.toRadians(90.00)), Math.toRadians(90.00))
                                .lineTo(new Vector2d(0.00, -47))
                                .splineToConstantHeading(new Vector2d(-25, -44), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-40, -28, Math.toRadians(180)), Math.toRadians(180.00))
                                .lineToLinearHeading(new Pose2d(-60, -55, Math.toRadians(245.00)))
                                .lineToLinearHeading(new Pose2d(-58, -28, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-60, -55, Math.toRadians(245.00)))
                                .lineToLinearHeading(new Pose2d(-68, -28, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-60, -55, Math.toRadians(245.00)))
                                .build()
                );

        meepMeep.setBackground(ImageIO.read(new File("/Users/prathyet/Downloads/field-into-the-deep.png")))
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }
}