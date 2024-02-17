package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

    public class MeepMeepTesting {
        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(800);
            Pose2d StartingPositionBlueFront = new Pose2d(12, 60, Math.toRadians(90));
            Pose2d StartingPositionBlueBack = new Pose2d(-36, 60, Math.toRadians(90));
            Pose2d StartingPositionRedBack = new Pose2d(-36, -60, Math.toRadians(-90));












            RoadRunnerBotEntity blueBackCenter = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionBlueBack)
                                    .lineTo(new Vector2d(-36, 30))
                                    //release pixel
                                    .forward(3)
                                    .strafeLeft(15)
                                    .lineTo(new Vector2d(-48, 12))
                                    .lineTo(new Vector2d(36, 12))
                                    .lineToLinearHeading(new Pose2d(50,35, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)))
                                    .lineTo(new Vector2d(60,60))
                                    .build()
                    );

            RoadRunnerBotEntity blueBackLeft2 = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionBlueBack)
                                    .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(180)))
                                    //release pixel
                                    .forward(3)
                                    .strafeRight(15)
                                    .lineToLinearHeading(new Pose2d(50,42, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)))
                                    .lineTo(new Vector2d(60,60))
                                    .build()
                    );

            RoadRunnerBotEntity redBackLeft = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionRedBack)
                                    .lineToLinearHeading(new Pose2d(-36, -34, Math.toRadians(0)))
                                    //release pixel
                                    .back(3)
                                    .waitSeconds(1)
                                    .forward(3)
                                    .lineToLinearHeading(new Pose2d(-36,-12, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(50,-28, Math.toRadians(0)))
                                    .back(3)
                                    .lineToLinearHeading(new Pose2d(47, -12, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
                                    .build()
                    );

            RoadRunnerBotEntity redBackCenter = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionRedBack)
                                    .lineToLinearHeading(new Pose2d(-36, -34, Math.toRadians(-90)))
                                    //release pixel
                                    .back(3)
                                    .waitSeconds(1)
                                    .forward(3)
                                    .strafeRight(15)
                                    .lineToLinearHeading(new Pose2d(-36,-12, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(50,-35, Math.toRadians(0)))
                                    .back(3)
                                    .lineToLinearHeading(new Pose2d(47, -12, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(0)))
                                    .build()
                    );

            RoadRunnerBotEntity redBackRight = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionRedBack)
                                    .setTurnConstraint(5, 5)
                                    .back(10)
                                    .turn(Math.toRadians(45))
                                    .setTurnConstraint(5, 5)
                                    .turn(Math.toRadians(-90))
                                    .setTurnConstraint(5,5)
                                    .turn(Math.toRadians(45))
                                    .back(4)
                                    .back(18)
                                    .turn(Math.toRadians(-90))
                                    .resetConstraints()
                                    .back(4)
                                    .waitSeconds(0.5)
                                    .waitSeconds(0.5)
                                    .waitSeconds(1)
                                    .forward(7)
                                    .strafeRight(17)
                                    .turn(Math.toRadians(180))
                                    .forward(70)
                                    .strafeRight(17)
                                    .forward(17)
                                    .back(5)
                                    .strafeLeft(15)
                                    .forward(15)
                                    .build()


                    );

            RoadRunnerBotEntity blueBackRight = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionBlueBack)
                                    .setTurnConstraint(5, 5)
                                    .back(10)
                                    .turn(Math.toRadians(45))
                                    .setTurnConstraint(5, 5)
                                    .turn(Math.toRadians(-90))
                                    .setTurnConstraint(5,5)
                                    .turn(Math.toRadians(45))
                                    .back(4)
                                    //scn end
                                    .back(13)
                                    .turn(Math.toRadians(-90))
                                    .resetConstraints()
                                    .back(4)
                                    //floor
                                    .waitSeconds(0.5)
                                    .waitSeconds(0.5)
                                    .waitSeconds(1)
                                    .forward(5)
                                    .strafeRight(24)
                                    .forward(70)
                                    .strafeLeft(27)
                                    .forward(12)
                                    .back(5)
                                    .strafeRight(18)
                                    .forward(15)
                                    .build()


                    );

            RoadRunnerBotEntity blueBackLeft = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionBlueBack)
                                    .setTurnConstraint(5, 5)
                                    .back(10)
                                    .turn(Math.toRadians(45))
                                    .setTurnConstraint(5, 5)
                                    .turn(Math.toRadians(-90))
                                    .setTurnConstraint(5,5)
                                    .turn(Math.toRadians(45))
                                    .back(4)
                                    .back(18)
                                    //Beginning Sequence
                                    .turn(Math.toRadians(90))
                                    .resetConstraints()
                                    .back(4)
                                    .waitSeconds(0.5)
                                    .waitSeconds(0.5)
                                    .waitSeconds(1)
                                    .forward(7)
                                    .strafeLeft(17)
                                    .turn(Math.toRadians(180))
                                    .forward(70)
                                    .strafeLeft(31)
                                    .forward(17)
                                    .back(5)
                                    .strafeRight(30)
                                    .forward(15)


                                    .build()


                    );

            RoadRunnerBotEntity blueFrontLeft = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionBlueFront)
                                    /*
                                    .lineToLinearHeading(new Pose2d(15, 30, Math.toRadians(180)))
                                    .waitSeconds(1)
                                    .forward(3)
                                    .waitSeconds(1)
                                    .strafeRight(15)
                                    .lineToLinearHeading(new Pose2d(50,42, Math.toRadians(0)))
                                    .waitSeconds(2)
                                    .waitSeconds(2)
                                    .back(3)
                                    .lineToLinearHeading(new Pose2d(47, 60, Math.toRadians(0)))
                                    .lineTo(new Vector2d(60,60))
                                    .build()
                                    */
                                    .setTurnConstraint(5, 5)
                                    .back(10)
                                    .turn(Math.toRadians(45))
                                    .setTurnConstraint(5, 5)
                                    .turn(Math.toRadians(-90))
                                    .setTurnConstraint(5,5)
                                    .turn(Math.toRadians(45))
                                    .back(4)
                                    .back(18)
                                    .turn(Math.toRadians(90))
                                    .resetConstraints()
                                    .back(4)
                                    .waitSeconds(0.5)
                                    .waitSeconds(0.5)
                                    .waitSeconds(1)
                                    .forward(7)
                                    .strafeRight(15)
                                    .turn(Math.toRadians(180))
                                    .forward(40)
                                    .waitSeconds(1.5)
                                    .back(5)
                                    .strafeLeft(15)
                                    .forward(15)
                                    .build()
                    );

            RoadRunnerBotEntity blueFrontRight = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionBlueFront)
                                    /*
                                    .lineToLinearHeading(new Pose2d(9, 30, Math.toRadians(0)))
                                    //release pixel
                                    .lineToLinearHeading(new Pose2d(50,28, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)))
                                    .lineTo(new Vector2d(60,60))
                                    .build()
                                     */
                                    .setTurnConstraint(5, 5)
                                    .back(10)
                                    .turn(Math.toRadians(45))
                                    .setTurnConstraint(5, 5)
                                    .turn(Math.toRadians(-90))
                                    .setTurnConstraint(5,5)
                                    .turn(Math.toRadians(45))
                                    .back(4)
                                    .back(18)
                                    .turn(Math.toRadians(-90))
                                    .resetConstraints()
                                    .back(4)
                                    //drop on floor
                                    .waitSeconds(1)
                                    .forward(40)
                                    //Drop Backboard
                                    .back(4)
                                    .strafeLeft(30)
                                    .forward(15)
                                    .build()
                    );

            RoadRunnerBotEntity blueFrontCenter = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(35, 35, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 16.3)
                    .setDimensions(15,15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(StartingPositionBlueFront)
                                    /*
                                    .lineTo(new Vector2d(12, 32))
                                    //release pixel
                                    .forward(3)
                                    .lineToLinearHeading(new Pose2d(50,35, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)))
                                    .lineTo(new Vector2d(60,60))
                                    .build()
                                    */
                                    .setTurnConstraint(5, 5)
                                    .back(10)
                                    .turn(Math.toRadians(45))
                                    .setTurnConstraint(5, 5)
                                    .turn(Math.toRadians(-90))
                                    .setTurnConstraint(5,5)
                                    .turn(Math.toRadians(45))
                                    .back(4)
                                    .back(14)
                                    //floor pixel
                                    .waitSeconds(1)
                                    .forward(4)
                                    .turn(Math.toRadians(-90))
                                    .resetConstraints()
                                    .forward(40)
                                    //backboard pixel
                                    .back(5)
                                    .strafeLeft(22)
                                    .forward(15)
                                    .build()
                    );







            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    //.addEntity(blueFrontCenter)
                    //.addEntity(blueFrontRight)
                    .addEntity(blueBackRight)
                    //.addEntity(blueFrontLeft)
                    //.addEntity(redBackLeft)
                    .start();
        }
    }