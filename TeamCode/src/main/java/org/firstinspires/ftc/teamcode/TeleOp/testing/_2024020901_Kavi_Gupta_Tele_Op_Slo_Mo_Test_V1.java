package org.firstinspires.ftc.teamcode.TeleOp.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class _2024020901_Kavi_Gupta_Tele_Op_Slo_Mo_Test_V1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer Localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        Localizer.setPoseEstimate(PoseStorage.currentPose);

        boolean Is_Robot_In_Slo_Mo = false;

        long Robot_Slo_Mo_Start_Time = 0;

        TrajectorySequence Move_Robot_Forward_For_Slo_Mo = null;

        while (opModeIsActive()) {
            Localizer.update();
            Pose2d Current_Robot_Pose = Localizer.getPoseEstimate();
            if (gamepad1.dpad_up) {
                if (Is_Robot_In_Slo_Mo == false) {
                    Move_Robot_Forward_For_Slo_Mo = MecanumDrivebase.trajectorySequenceBuilder(Current_Robot_Pose)
                            //.setVelConstraint(MecanumDrivebaseInstance.getVelocityConstraint(5, 10, DriveConstants.TRACK_WIDTH))
                            .forward(2)
                            .build();
                    Robot_Slo_Mo_Start_Time = System.currentTimeMillis();
                    MecanumDrivebase.followTrajectorySequenceAsync(Move_Robot_Forward_For_Slo_Mo);
                    Is_Robot_In_Slo_Mo = true;
                }
            }

            if (Is_Robot_In_Slo_Mo) {
                if (System.currentTimeMillis() >= Robot_Slo_Mo_Start_Time + Move_Robot_Forward_For_Slo_Mo.duration()) {
                    Is_Robot_In_Slo_Mo = false;
                }
            }

            telemetry.addData("Is Robot In Slo Mo: ", Is_Robot_In_Slo_Mo);
            telemetry.addData("Robot Slo-Mo Start Time: ", Robot_Slo_Mo_Start_Time);
            telemetry.addData("Slo Mo Trajectory Duration: ", Move_Robot_Forward_For_Slo_Mo.duration());

            telemetry.addLine();

            telemetry.addData("Robot X: ", Current_Robot_Pose.getX());
            telemetry.addData("Robot Y: ", Current_Robot_Pose.getX());
            telemetry.addData("Robot Heading: ", Current_Robot_Pose.getHeading());
            telemetry.update();
        }
    }
}
