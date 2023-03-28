package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double dropTime = .7;
        double pickupTime = .9;


        RoadRunnerBotEntity myBotRight2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(35, -65, Math.toRadians(90)))
                                        .splineToConstantHeading(new Vector2d(59, -57), Math.toRadians(90))
                                        .forward(30)
                                        .splineTo(new Vector2d(22, -16), Math.toRadians(-180))
                                        .build());

        RoadRunnerBotEntity myBotRight3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -65, Math.toRadians(-180)))
                                .strafeTo(new Vector2d(33, -23))
                                //.strafeRight(42)
                                //.waitSeconds(.2)
//                                .strafeTo(new Vector2d(43, -8))
                                .splineToConstantHeading(new Vector2d(45, -8), Math.toRadians(-180))
                                .back(10)
//                                .forward(10)
//                                .splineToConstantHeading(new Vector2d(33, -23), Math.toRadians(-180))
                                .build());

        RoadRunnerBotEntity myBotRight4 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(33, -23, Math.toRadians(-180)))
//                                .strafeTo(new Vector2d(33, -23))
                                //.strafeRight(42)
                                //.waitSeconds(.2)
//                                .strafeTo(new Vector2d(43, -8))
                                .splineToConstantHeading(new Vector2d(45, -8), Math.toRadians(0))
                                .back(10)
//                                .forward(10)
//                                .splineToConstantHeading(new Vector2d(33, -23), Math.toRadians(-180))
                                .build());

        RoadRunnerBotEntity myBotRight5 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(33, -65, Math.toRadians(0)))
                                .waitSeconds(.9)
                                .strafeTo(new Vector2d(32, -23))
                                .strafeRight(15)
                                .waitSeconds(.9)
                                .back(25)
                                .strafeTo(new Vector2d(30, -10))
                                .build());

        RoadRunnerBotEntity myBotRight6 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(33, -65, Math.toRadians(0)))
                                .strafeTo(new Vector2d(32, -23))
                                .strafeRight(15)
                                .back(25)
                                .strafeTo(new Vector2d(30, -10))
                                .build());

        RoadRunnerBotEntity myBotRight7 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12, 18)
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(-33, -65, Math.toRadians(0)))
                .strafeTo(new Vector2d(-31, -22))
                .strafeLeft(13)
                .strafeTo(new Vector2d(-57, -12))
//                .strafeRight(4)
//                .back(26)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(myBotLeft)
                //.addEntity(myBotLeft2)
                //.addEntity(myBotRight5)
                //.addEntity(myBotRight6)
                .addEntity(myBotRight7)
                .start();
    }
}