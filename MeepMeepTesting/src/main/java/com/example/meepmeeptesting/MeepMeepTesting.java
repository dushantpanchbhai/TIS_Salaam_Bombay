package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String args[])
    {
        MeepMeep meepMeep = new MeepMeep(800);

        double y = 8.66;
        Pose2d start = new Pose2d(23,57.5+y,Math.toRadians(-90));

        double x1 = 23;
        double y1 = 57.5+y;

        double xoffset = 0;
        double yoffset = 0;
        double x2 = -11.5+xoffset;
        double y2 = 23+yoffset;

        //distance between point and carousel;
//        double frontDist = Math.sqrt(Math.pow(Math.abs(x1-x2),2) + Math.pow(Math.abs(y1-y2),2));
        double frontDist = 30;
        double frontAngle = Math.toDegrees(Math.atan((double) Math.abs(y2-y1)/(double) Math.abs(x2-x1)));

        //distance between top and bottom point
        double backDist = 40.25-23;
        double backAngle = 90 - frontAngle;


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .forward(y)
//                                .turn(Math.toRadians(-frontAngle))
                                .turn(Math.toRadians(-45))
                                .forward(frontDist)
                                .back(frontDist)
//                                .turn(Math.toRadians(-(90-frontAngle)))
                                .turn(Math.toRadians(-45))
                                .back(backDist)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}