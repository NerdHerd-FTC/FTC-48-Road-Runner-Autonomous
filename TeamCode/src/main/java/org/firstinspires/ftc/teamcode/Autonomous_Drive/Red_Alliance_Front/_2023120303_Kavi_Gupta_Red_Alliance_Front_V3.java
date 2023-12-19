package org.firstinspires.ftc.teamcode.Autonomous_Drive.Red_Alliance_Front;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.tensorFlow.TensorFlowInstance;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstance;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled

public class _2023120303_Kavi_Gupta_Red_Alliance_Front_V3 extends LinearOpMode {
    @Override
    public void runOpMode() {

        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        TensorFlowInstance TensorFlow = new TensorFlowInstance();
        ClawInstance Claw = new ClawInstance();
        ArmInstance Arm = new ArmInstance();
        final TensorFlowInstance.CameraStreamProcessor processor = new TensorFlowInstance.CameraStreamProcessor();

        Pose2d StartingCoordinates = new Pose2d(12, -60, Math.toRadians(270));
        MecanumDrivebase.setPoseEstimate(StartingCoordinates);
        TensorFlow.IntitializeTensorFlow(hardwareMap, processor);
        Claw.initializeClaw(hardwareMap);
        Arm.initializeArm(hardwareMap);

        String PropLocation;
        TensorFlow.SetWebcamStreamStatus("start");


        TrajectorySequence MoveToPropScanningLocation = MecanumDrivebase.trajectorySequenceBuilder(StartingCoordinates)
                .addDisplacementMarker(() -> {
                    Arm.moveArmTo(200);
                    Claw.Actuate_Claw_Top_Finger("close");
                    Claw.Actuate_Claw_Bottom_Finger("close");
                })
                .back(12)
                .build();

        TrajectorySequence PlacePixelOnCenterSpike = MecanumDrivebase.trajectorySequenceBuilder(StartingCoordinates)
                .strafeLeft(5)
                .back(15)
                .addDisplacementMarker(() -> {
                    //claw release
                    Arm.moveArmTo(10);
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(5)
                .build();
        TrajectorySequence PlacePixelOnLeftSpike = MecanumDrivebase.trajectorySequenceBuilder(StartingCoordinates)
                .back(12)
                .turn(Math.toRadians(90))
                .strafeLeft(5)
                .forward(12)
                .addDisplacementMarker(() -> {
                    //claw release
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(3)
                .build();
        TrajectorySequence PlaceYellowPixelOnCenterBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnCenterSpike.end())
                .forward(5)
                .strafeLeft(20)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    Arm.moveArmTo(450);
                })
                .forward(5)
                .waitSeconds(3)
                .addDisplacementMarker(() -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(2)
                .back(5)
                .strafeRight(30)
                .build();

        TrajectorySequence PlaceYellowPixelOnLeftBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnLeftSpike.end())
                .setReversed(true)
                .back(30)
                .strafeLeft(10)
                .addDisplacementMarker(() -> {
                    Arm.moveArmTo(450);
                })
                .waitSeconds(3)
                .addDisplacementMarker(() -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(2)
                .back(5)
                .strafeRight(35)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        MecanumDrivebase.followTrajectorySequence(MoveToPropScanningLocation);

        TensorFlow.SetWebcamStreamStatus("start");

        PropLocation = "center";

        if (PropLocation == "left") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnLeftSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnLeftBackDrop);
        } else if (PropLocation == "center") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnCenterSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnCenterBackDrop);

        } else if (PropLocation == "right") {
            //do
        } else {
            //failsafe
        }




        //Scan for props

    }
}
