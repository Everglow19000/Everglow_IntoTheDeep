package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class speciminsPreload {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14,18.1)
                .build();

        double BasketY = -34;
        double BasketHeading = Math.PI/2;
        Pose2d specimins_beginPose = new Pose2d(31.1, -63,   Math.PI/2);
        Pose2d specimins_dropPose = new Pose2d(31.1, -57,   0);
        Pose2d specimins_endPose = new Pose2d(63,-60,Math.PI/2);


        // Start with the vertical elevator extended to the required height
        // Start with the horizontal elevator already extended as needed
        // The horizontal elevator could always be extended to the right amount
        TrajectoryActionBuilder specimins_path_Preload = myBot.getDrive().actionBuilder(specimins_beginPose)
                .waitSeconds(1)

                // Raise the vertical elevator while moving
                .strafeToSplineHeading(new Vector2d(-5,BasketY),BasketHeading)

                .waitSeconds(0.1)
                //when finished raising move forward
                .lineToY(BasketY+1)

                //drop specimin

                // Lower the vertical elevator while moving
                .waitSeconds(1)
                .strafeToSplineHeading(specimins_dropPose.position, specimins_dropPose.heading)

                // Pick up specimin

                .waitSeconds(1)
                // Raise the vertical elevator while moving
                .strafeToSplineHeading(new Vector2d(-5,BasketY),BasketHeading)

                .waitSeconds(0.1)
                //when finished raising move forward
                .lineToY(BasketY+1)

                //Drop specimin

                .waitSeconds(1)
                //lower elevator while moving
                .strafeToSplineHeading(specimins_endPose.position,specimins_endPose.heading)
                ;

//
//        TrajectoryActionBuilder specimins_path= myBot.getDrive().actionBuilder(specimins_dropPose)
//                // Moves the first sample
////                .waitSeconds(1)
//                .setTangent((1)*Math.PI / 2)
//                .splineToLinearHeading(new Pose2d(33,-30, Math.PI / 2),Math.PI / 2)
//                .splineToSplineHeading(new Pose2d(firstSpeciminX,PickSpeciminY, 0),0)
//                .strafeTo(new Vector2d(firstSpeciminX,dropSpeciminY))
//                // Moves the second sample
//                .setTangent(Math.PI * 0.5)
//                .splineToConstantHeading(new Vector2d(secondSpeciminX,PickSpeciminY),0)
//                .strafeTo(new Vector2d(secondSpeciminX,dropSpeciminY))
//
//                // Goes to the basket
//                .setTangent((1) * Math.PI)
//                .strafeToSplineHeading(specimins_basketPose.position, specimins_basketPose.heading)
//                .strafeTo(new Vector2d(specimins_basketPose.position.x, specimins_basketPose.position.y + 10))
//                .strafeTo(specimins_basketPose.position)
//                // Dropping the sample -Lowering the vertical elevator
//                .strafeToSplineHeading(specimins_dropPose.position, specimins_dropPose.heading)
//                // Dropping the sample - Lowering the vertical elevator
//                // Dropping the sample - releasing the sample
//
//                // Waits for player to pick up the sample
//                .strafeTo(new Vector2d(specimins_dropPose.position.x + 10, specimins_dropPose.position.y))
//                .strafeTo(specimins_dropPose.position)
//                // Picks up the sample
//                // While moving extending the vertical elevator
//                .splineToLinearHeading(new Pose2d(0,-34, Math.PI/2),Math.PI/2)
//                .strafeToSplineHeading(specimins_basketPose.position, specimins_basketPose.heading)
//                .strafeTo(new Vector2d(specimins_basketPose.position.x, specimins_basketPose.position.y + 10))
//                .strafeTo(specimins_basketPose.position)
//                // Goes to to drop the specimen
//                .strafeToSplineHeading(specimins_dropPose.position, specimins_dropPose.heading)
//                // Dropping the sample - Lowering the vertical elevator
//                // Dropping the sample - releasing the sample
//                // Waits for player to pick up the sample
//                .strafeTo(new Vector2d(specimins_dropPose.position.x + 10, specimins_dropPose.position.y))
//                .strafeTo(specimins_dropPose.position)
//                // Goes to the basket
//                .waitSeconds(0.1)
//                .setTangent((1) * Math.PI)
//                .strafeToSplineHeading(specimins_basketPose.position, specimins_basketPose.heading)
//                .strafeTo(new Vector2d(specimins_basketPose.position.x, specimins_basketPose.position.y + 10))
//                .strafeTo(specimins_basketPose.position)
//                // Dropping the sample -Lowering the vertical elevator
//                .strafeTo(new Vector2d(38, -34))
//                .splineToLinearHeading((specimins_endPose),Math.PI)
//                ;

        myBot.runAction(specimins_path_Preload.build());




































        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
