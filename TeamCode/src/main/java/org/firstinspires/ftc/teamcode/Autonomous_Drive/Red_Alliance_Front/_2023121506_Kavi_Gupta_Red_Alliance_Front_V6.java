package org.firstinspires.ftc.teamcode.Autonomous_Drive.Red_Alliance_Front;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.tensorFlow.TensorFlowInstance_Individual_Scanning;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstance;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Autonomous
public class _2023121506_Kavi_Gupta_Red_Alliance_Front_V6 extends LinearOpMode {
    @Override
    public void runOpMode() {

        int ArmMovementValue = 555;
        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        TensorFlowInstance_Individual_Scanning TensorFlow = new TensorFlowInstance_Individual_Scanning();
        ClawInstance Claw = new ClawInstance();
        ArmInstance Arm = new ArmInstance();
        final TensorFlowInstance_Individual_Scanning.CameraStreamProcessor processor = new TensorFlowInstance_Individual_Scanning.CameraStreamProcessor();

        Pose2d StartingCoordinates = new Pose2d(12, -60, Math.toRadians(270));
        MecanumDrivebase.setPoseEstimate(StartingCoordinates);
        TensorFlow.IntitializeTensorFlow(hardwareMap, processor);
        Claw.initializeClaw(hardwareMap);
        Arm.initializeArm(hardwareMap);

        double leftDetectionConfidence = 0;
        double centerDetectionConfidence = 0;
        double rightDetectionConfidence = 0;

        List<Double> detectionConfidences = new ArrayList<Double>();
        String PropLocation = null;
        TensorFlow.SetWebcamStreamStatus("start");

        TrajectorySequence RotateToLeftProp = MecanumDrivebase.trajectorySequenceBuilder(StartingCoordinates)
                .setTurnConstraint(5, 5)
                .back(10)
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence RotateToRightProp = MecanumDrivebase.trajectorySequenceBuilder(RotateToLeftProp.end())
                .setTurnConstraint(5, 5)
                .turn(Math.toRadians(-90))
                .build();


        TrajectorySequence MoveToPropScanningLocation = MecanumDrivebase.trajectorySequenceBuilder(RotateToRightProp.end())
                .setTurnConstraint(5,5)
                .turn(Math.toRadians(45))
                .back(4)
                .build();

        TrajectorySequence PlacePixelOnCenterSpike = MecanumDrivebase.trajectorySequenceBuilder(MoveToPropScanningLocation.end())
                .strafeLeft(4)
                .back(16)
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmMovementValue);
                })
                .waitSeconds(1)
                .build();


        TrajectorySequence PlacePixelOnLeftSpike = MecanumDrivebase.trajectorySequenceBuilder(MoveToPropScanningLocation.end())
                .back(18)
                .turn(Math.toRadians(90))
                .back(4)
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmMovementValue);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence PlacePixelOnRightSpike = MecanumDrivebase.trajectorySequenceBuilder(MoveToPropScanningLocation.end())
                .back(18)
                .turn(Math.toRadians(-90))
                .back(4)
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmMovementValue);
                })
                .waitSeconds(1)
                .build();



        TrajectorySequence PlaceYellowPixelOnCenterBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnCenterSpike.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmMovementValue);
                })
                .setVelConstraint(MecanumDrivebase.getVelocityConstraint(8, 8, DriveConstants.TRACK_WIDTH))
                .forward(4)
                .strafeLeft(15)
                .turn(Math.toRadians(90))
                .forward(15)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .build();

        TrajectorySequence PlaceYellowPixelOnLeftBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnLeftSpike.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmMovementValue);
                })
                .setVelConstraint(MecanumDrivebase.getVelocityConstraint(8, 8, DriveConstants.TRACK_WIDTH))
                .strafeLeft(3)
                .forward(42)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .build();

        TrajectorySequence PlaceYellowPixelOnRightBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnRightSpike.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmMovementValue);
                })
                .setVelConstraint(MecanumDrivebase.getVelocityConstraint(10, 10, DriveConstants.TRACK_WIDTH))
                .forward(7)
                .turn(Math.toRadians(180))
                .strafeRight(18)
                .forward(42)
                .strafeLeft(5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .build();


        TrajectorySequence FailsafePark = MecanumDrivebase.trajectorySequenceBuilder(MoveToPropScanningLocation.end())
                .forward(3)
                .strafeLeft(40)
                .build();

        TrajectorySequence ParkFromCenterBoardPlacement = MecanumDrivebase.trajectorySequenceBuilder(PlaceYellowPixelOnCenterBackDrop.end())
            .back(5)
            .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                Arm.moveArmTo(50);
            })
            .back(4)
            .strafeRight(25)
            .forward(12)
            .resetVelConstraint()
            .build();

        TrajectorySequence ParkFromLeftBoardPlacement = MecanumDrivebase.trajectorySequenceBuilder(PlaceYellowPixelOnLeftBackDrop.end())
            .back(5)
            .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    Arm.moveArmTo(50);
            })
            .strafeRight(30)
            .forward(12)
            .resetVelConstraint()
            .build();

        TrajectorySequence ParkFromRightBoardPlacement = MecanumDrivebase.trajectorySequenceBuilder(PlaceYellowPixelOnRightBackDrop.end())
            .back(5)
            .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                Arm.moveArmTo(50);
            })
            .strafeRight(17)
            .forward(12)
            .build();


        waitForStart();

        if (isStopRequested()) return;

        Claw.Actuate_Claw_Bottom_Finger("close");
        Claw.Actuate_Claw_Top_Finger("close");

        Arm.moveArmTo(300);

        MecanumDrivebase.followTrajectorySequence(RotateToLeftProp);
        sleep(1250);
        leftDetectionConfidence = TensorFlow.DetectProps();
        detectionConfidences.add(leftDetectionConfidence);
        sleep(750);



        MecanumDrivebase.followTrajectorySequence(RotateToRightProp);
        sleep(1250);
        rightDetectionConfidence = TensorFlow.DetectProps();
        detectionConfidences.add(rightDetectionConfidence);
        sleep(750);



        MecanumDrivebase.followTrajectorySequence(MoveToPropScanningLocation);
        sleep(1250);
        centerDetectionConfidence = TensorFlow.DetectProps();
        detectionConfidences.add(centerDetectionConfidence);
        sleep(750);

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

        TensorFlow.SetWebcamStreamStatus("start");
        if (PropLocation == "left") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnLeftSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnLeftBackDrop);
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            MecanumDrivebase.followTrajectorySequence(ParkFromLeftBoardPlacement);
        } else if (PropLocation == "center") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnCenterSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnCenterBackDrop);
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            MecanumDrivebase.followTrajectorySequence(ParkFromCenterBoardPlacement);

        } else if (PropLocation == "right") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnRightSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnRightBackDrop);
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            MecanumDrivebase.followTrajectorySequence(ParkFromRightBoardPlacement);
        } else {
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            Arm.moveArmTo(50);
            MecanumDrivebase.followTrajectorySequence(FailsafePark);
        }


        PoseStorage.currentPose = MecanumDrivebase.getPoseEstimate();

        //Scan for props

    }
}
