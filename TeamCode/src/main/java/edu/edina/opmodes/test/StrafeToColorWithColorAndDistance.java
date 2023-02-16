package edu.edina.opmodes.test;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class StrafeToColorWithColorAndDistance extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ColorSensor backColor = hardwareMap.get(ColorSensor.class, "backColor");
        ColorSensor frontColor = hardwareMap.get(ColorSensor.class, "frontColor");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        float hsvValues[] = {0F,0F,0F};
        NormalizedRGBA colors;

        drive.setPoseEstimate(new Pose2d(36, 64, Math.toRadians(270)));
        TrajectorySequence strafe = drive.trajectorySequenceBuilder(new Pose2d(36, 64, Math.toRadians(270)))
                .strafeRight(10).build();

        while (!isStarted()) {
            Color.RGBToHSV(backColor.red() * 8, backColor.green() * 8, backColor.blue() * 8, hsvValues);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Back Hue", "%.3f", hsvValues[0]);

            Color.RGBToHSV(frontColor.red() * 8, frontColor.green() * 8, frontColor.blue() * 8, hsvValues);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Front Hue", "%.3f", hsvValues[0]);

            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(strafe);
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            Color.RGBToHSV(backColor.red() * 8, backColor.green() * 8, backColor.blue() * 8, hsvValues);
            if ((hsvValues[0] < 100) || (hsvValues[0] > 160)){
                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                break;
            }

            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Back Hue", "%.3f", hsvValues[0]);
            telemetry.update();
            drive.update();
            idle();
        }

        double distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH);

        if (distanceToTravel > 4) {
            distanceToTravel = 4;
        }
        TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(2)
                .back(distanceToTravel)
                .build();

        drive.followTrajectorySequence(forward);

        while (!isStopRequested()) {
            Color.RGBToHSV(backColor.red() * 8, backColor.green() * 8, backColor.blue() * 8, hsvValues);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Back Hue", "%.3f", hsvValues[0]);

            Color.RGBToHSV(frontColor.red() * 8, frontColor.green() * 8, frontColor.blue() * 8, hsvValues);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Trajectory pose", strafe.end());
            telemetry.addLine()
                    .addData("Front Hue", "%.3f", hsvValues[0]);

            telemetry.update();
            idle();
        }
    }
}
