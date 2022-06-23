package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
    public static void main(String args[])
    {
        MeepMeep meepMeep = new MeepMeep(800);

        double x = 7;
        double y = 8.66;

        Pose2d start = new Pose2d(11.5+x,57.5+y,Math.toRadians(-90));

        //distance = sqrt((-11.5-11.5-x)^2 + (23-57.5-y)^2)
        double frontDist = Math.sqrt(Math.pow(-11.5-11.5-x,2) + Math.pow(23-57.5-y,2));
        double angle = 90 - Math.toDegrees(Math.atan((double) Math.abs(11.5+x+11.5)/(double) Math.abs(57.5+y-23)));
        double backDist = 23;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .turn(Math.toRadians(-angle))
                                .forward(frontDist)
                                .back(frontDist)
                                .turn(Math.toRadians(-(90-angle)))
                                .back(23)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}