package org.firstinspires.ftc.teamcode.Autonomous_Drive.Blue_Alliance_Front;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.tensorFlow.TensorFlowInstance_Individual_Scanning;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstancePrevious;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Autonomous
public class _2024023608_Kavi_Gupta_Blue_Alliance_Front_V8 extends LinearOpMode {
    @Override
    public void runOpMode() {

        final short ArmBackboardPosition = 580;
        final short ArmRestPosition = 150;
        final short ArmPlacePixelOnFloorPosition = 5;

        final MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        final TensorFlowInstance_Individual_Scanning TensorFlow = new TensorFlowInstance_Individual_Scanning();
        final ClawInstance Claw = new ClawInstance();
        final ArmInstancePrevious Arm = new ArmInstancePrevious();
        final TensorFlowInstance_Individual_Scanning.CameraStreamProcessor processor = new TensorFlowInstance_Individual_Scanning.CameraStreamProcessor();

        final Pose2d BlueFrontStartingCoordinates = new Pose2d(12, 60, Math.toRadians(90));
        MecanumDrivebase.setPoseEstimate(BlueFrontStartingCoordinates);
        TensorFlow.IntitializeTensorFlow(hardwareMap, processor);
        Claw.initializeClaw(hardwareMap);
        Arm.initializeArm(hardwareMap);

        double leftDetectionConfidence = 0;
        double centerDetectionConfidence = 0;
        double rightDetectionConfidence = 0;

        List<Double> detectionConfidences = new ArrayList<Double>();
        String PropLocation = null;
        //TensorFlow.SetWebcamStreamStatus("start");

        TrajectorySequence RotateToLeftProp = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontStartingCoordinates)
                .setTurnConstraint(5, 5)
                .back(10)
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence RotateToRightProp = MecanumDrivebase.trajectorySequenceBuilder(RotateToLeftProp.end())
                .setTurnConstraint(5, 5)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence RotateToCenterProp = MecanumDrivebase.trajectorySequenceBuilder(RotateToRightProp.end())
                .setTurnConstraint(5,5)
                .turn(Math.toRadians(45))
                .back(4)
                .build();

        TrajectorySequence BlueFrontCenter = MecanumDrivebase.trajectorySequenceBuilder(RotateToCenterProp.end())

                //scn end

                .back(13)
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                //floor pixel
                .waitSeconds(0.5)
                .back(6)
                .waitSeconds(0.5)
                .forward(8)
                .turn(Math.toRadians(-90))
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .forward(37)
                .turn(Math.toRadians(5))
                .waitSeconds(0.5)
                //backboard pixel
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.moveArmTo(50);
                })
                .waitSeconds(1)
                .back(5)
                .strafeLeft(25)
                .forward(12)
                .build();

        TrajectorySequence BlueFrontLeft = MecanumDrivebase.trajectorySequenceBuilder(RotateToCenterProp.end())

                //scn end
                .back(18)
                .turn(Math.toRadians(90))
                .resetConstraints()
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .waitSeconds(0.5)
                .back(4)
                .waitSeconds(0.5)
                .forward(5)
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .back(25)
                .turn(Math.toRadians(180))
                .forward(10)
                .strafeRight(2)
                .turn(Math.toRadians(-10))
                .forward(1)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.moveArmTo(50);
                })
                .waitSeconds(1)
                .back(5)
                .strafeRight(26)
                .forward(15)
                .build();

        TrajectorySequence BlueFrontRight = MecanumDrivebase.trajectorySequenceBuilder(RotateToCenterProp.end())

                //scn end
                .back(17)
                .turn(Math.toRadians(-90))
                .resetConstraints()
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                //drop on floor
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .waitSeconds(0.5)
                .back(4)
                .waitSeconds(0.5)
                .forward(42)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(565);
                })
                //Drop Backboard
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.moveArmTo(50);
                })
                .waitSeconds(1)
                .back(4)
                .strafeLeft(30)
                .forward(15)
                .build();

        TrajectorySequence Park = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontStartingCoordinates)
                .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)))
                .lineTo(new Vector2d(60,60))
                .build();









        waitForStart();

        Claw.Actuate_Claw_Top_Finger("close");
        Claw.Actuate_Claw_Bottom_Finger("close");


        if (opModeIsActive()) {
            Arm.moveArmTo(300);
            sleep(250);
            MecanumDrivebase.followTrajectorySequence(RotateToLeftProp);
            sleep(500);
            leftDetectionConfidence = TensorFlow.DetectProps();
            sleep(250);
            detectionConfidences.add(leftDetectionConfidence);
            MecanumDrivebase.followTrajectorySequence(RotateToRightProp);
            sleep(500);
            rightDetectionConfidence = TensorFlow.DetectProps();
            sleep(250);
            detectionConfidences.add(rightDetectionConfidence);
            MecanumDrivebase.followTrajectorySequence(RotateToCenterProp);
            sleep(500);
            centerDetectionConfidence = TensorFlow.DetectProps();
            sleep(250);
            detectionConfidences.add(centerDetectionConfidence);


            double HighestConfidenceRate = Collections.max(detectionConfidences);

            if (HighestConfidenceRate == leftDetectionConfidence) {
                PropLocation = "left";
            } else if (HighestConfidenceRate == rightDetectionConfidence) {
                PropLocation = "right";
            } else if (HighestConfidenceRate == centerDetectionConfidence) {
                PropLocation = "center";
            }

            telemetry.addLine("Prop Location: " + PropLocation);
            telemetry.update();



            Arm.moveArmTo(ArmRestPosition);
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            sleep(1000);
            Arm.moveArmTo(300);

            if ("center" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(BlueFrontCenter);
            } else if ("left" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(BlueFrontLeft);
            } else if ("right" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(BlueFrontRight);
            } else {
                //MecanumDrivebase.followTrajectorySequence(Park);
            }
            while (Arm.Arm_Motor.isBusy()) {};
        }

        PoseStorage.currentPose = MecanumDrivebase.getPoseEstimate();

    }
}
