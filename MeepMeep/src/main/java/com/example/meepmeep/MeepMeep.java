package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.63)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                .forward(14)
                                .strafeRight(11)
                                .waitSeconds(2)
                                .strafeLeft(12)
                                .waitSeconds(1)
                                .forward(13)
                                .turn(Math.toRadians(270))
                                .back(8)
                                .build()
                );
                                /*.splineTo(new Vector2d(37, -51), Math.toRadians(90))
                                .forward(16)
                                .waitSeconds(2)
                                .strafeLeft(11)
                                .forward(10)
                                .waitSeconds(1)
                                .forward(3)
                                .turn(Math.toRadians(90))
                                .strafeLeft(8)
                                .splineTo(new Vector2d(60, -60), 0)
                                .build()
                );*/

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}