package edu.edina.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.RobotState;
import edu.edina.library.vision.AprilTagDetectionPipeline;

@Autonomous(group = "Simple")@Config
public class LeftSidePark extends AutoBase {

    protected String getCameraName() {
        return "lowCamera";
    }

    @Override
    protected void addAdditionalTelemetry(Telemetry telemetry) {
        telemetry.addData("Make sure claw is in the front and low camera is facing field.", "");
        telemetry.addData("Cone should always be on side with medium pole", "");
    }

    protected boolean shouldClawBeInTheFront() {
        return true;
    }

    protected Pose2d getStartPose() { return new Pose2d(-33, -65, Math.toRadians(0)); }

    @Override
    public void initPaths() {
        Vector2d startEndPoint = new Vector2d(-30, -22);
        // cone one drop off
        start = drive.trajectorySequenceBuilder(new Pose2d(-33, -65, Math.toRadians(0)))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.POLEPOSITIONLOW);
                })
                .addTemporalMarker(1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .strafeTo(startEndPoint)
                .build();

        // park
        backToPickup6_left = drive.trajectorySequenceBuilder(start.end())
                .addTemporalMarker(0.8, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .strafeRight(13)
                .back(25.5)
                .build();

        backToPickup6_middle = drive.trajectorySequenceBuilder(start.end())
                .addTemporalMarker(0.8, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .strafeRight(13)
                .build();

        backToPickup6_right = drive.trajectorySequenceBuilder(start.end())
                .addTemporalMarker(0.8, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .strafeRight(13)
                .forward(20)
                .build();
    }

    @Override
    protected void runPaths() {

        drive.setPoseEstimate(getStartPose());

        drive.followTrajectorySequence(start);
    }
}
