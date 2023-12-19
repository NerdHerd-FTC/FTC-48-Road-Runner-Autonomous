package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity blueFrontCenter = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                                .setTurnConstraint(5, 5)
                                .back(10)
                                .turn(Math.toRadians(-45))
                                .setTurnConstraint(5, 5)
                                .turn(Math.toRadians(90))
                                .setTurnConstraint(5,5)
                                .turn(Math.toRadians(-45))
                                .back(4)
                                .strafeRight(4)
                                .back(16)
                                .addTemporalMarker(() -> {
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker( () -> {
                                })
                                .waitSeconds(1)
                                .forward(4)
                                .strafeRight(15)
                                .turn(Math.toRadians(-90))
                                .forward(15)
                                .build()
                );

        RoadRunnerBotEntity redFrontCenter = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                                .setTurnConstraint(5, 5)
                                .back(10)
                                .turn(Math.toRadians(45))
                                .setTurnConstraint(5, 5)
                                .turn(Math.toRadians(-90))
                                .setTurnConstraint(5,5)
                                .turn(Math.toRadians(45))
                                .back(4)
                                .strafeLeft(4)
                                .back(16)
                                .forward(4)
                                .strafeLeft(15)
                                .turn(Math.toRadians(90))
                                .forward(15)
                                .build()
                );

        RoadRunnerBotEntity redFrontLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                                .setTurnConstraint(5, 5)
                                .back(10)
                                .turn(Math.toRadians(45))
                                .setTurnConstraint(5, 5)
                                .turn(Math.toRadians(-90))
                                .setTurnConstraint(5,5)
                                .turn(Math.toRadians(45))
                                .back(4)
                                //new
                                .back(18)
                                .turn(Math.toRadians(90))
                                .back(4)
                                .strafeLeft(3)
                                .forward(42)
                                .build()
                );
        RoadRunnerBotEntity blueFrontRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                                .setTurnConstraint(5, 5)
                                .back(10)
                                .turn(Math.toRadians(45))
                                .setTurnConstraint(5, 5)
                                .turn(Math.toRadians(-90))
                                .setTurnConstraint(5,5)
                                .turn(Math.toRadians(45))
                                .back(4)
                                //spike
                                .back(18)
                                .turn(Math.toRadians(-90))
                                .back(4)
                                //backdrop
                                .strafeRight(3)
                                .forward(42)
                                .build()
                );

        RoadRunnerBotEntity redFrontRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                                .setTurnConstraint(5, 5)
                                .back(10)
                                .turn(Math.toRadians(45))
                                .setTurnConstraint(5, 5)
                                .turn(Math.toRadians(-90))
                                .setTurnConstraint(5,5)
                                .turn(Math.toRadians(45))
                                .back(4)
                                //spike
                                .back(18)
                                .turn(Math.toRadians(-90))
                                .back(4)
                                //backdrop
                                .forward(7)
                                .turn(Math.toRadians(180))
                                .strafeRight(18)
                                .forward(42)
                                .strafeLeft(5)
                                .build()
                );

        RoadRunnerBotEntity blueFrontLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                                .setTurnConstraint(5, 5)
                                .back(10)
                                .turn(Math.toRadians(45))
                                .setTurnConstraint(5, 5)
                                .turn(Math.toRadians(-90))
                                .setTurnConstraint(5,5)
                                .turn(Math.toRadians(45))
                                .back(4)
                                //spike
                                .back(18)
                                .turn(Math.toRadians(90))
                                .back(4)
                                //backdrop
                                .forward(7)
                                .turn(Math.toRadians(-180))
                                .strafeLeft(18)
                                .forward(42)
                                .strafeRight(5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(blueFrontCenter)
                //.addEntity(redFrontCenter)
                //.addEntity(redFrontLeft)
                //.addEntity(blueFrontLeft)
                .addEntity(blueFrontRight)
                .addEntity(redFrontRight)
                .start();
    }
}