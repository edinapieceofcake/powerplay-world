package edu.edina.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(group = "Left")
@Config
public class LeftSideHigh extends AutoBase {
    @Override
    protected String getCameraName() {
        return "highCamera";
    }

    @Override
    protected void addAdditionalTelemetry(Telemetry telemetry) {
        telemetry.addData("Make sure claw is in the front and high camera is facing field.", "");
        telemetry.addData("Cone should always be on side with medium pole", "");
    }

    @Override
    protected boolean shouldClawBeInTheFront() {
        return true;
    }

    @Override
    protected void initPaths() {
        // cone one drop off
        drive.setPoseEstimate(new Pose2d(-33, -65, Math.toRadians(0)));

        start = drive.trajectorySequenceBuilder(new Pose2d(-33, -65, Math.toRadians(0)))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.POLEPOSITIONLOW);
                })
                .addTemporalMarker(1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .strafeTo(new Vector2d(-31, -19))
                .build();

        // cone two pickup
        backToPickup1 = drive.trajectorySequenceBuilder(start.end())
                .strafeLeft(11)
                .strafeTo(new Vector2d(-55, -13))
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(.7, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION5);
                })
                .addTemporalMarker(2.9, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone two dropoff
        backToDropOff1 = drive.trajectorySequenceBuilder(backToPickup1.end())
                .strafeTo(new Vector2d(-20, -6))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWLEFTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();

        // cone three pickup
        backToPickup2 = drive.trajectorySequenceBuilder(backToDropOff1.end())
                .strafeTo(new Vector2d(-55, -13))
                .addTemporalMarker(.1, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(.3, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION4);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone three dropoff
        backToDropOff2 = drive.trajectorySequenceBuilder(backToPickup2.end())
                .strafeTo(new Vector2d(-20, -6))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWLEFTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();

        // cone four pickup
        backToPickup3 = drive.trajectorySequenceBuilder(backToDropOff2.end())
                .strafeTo(new Vector2d(-55, -13))
                .addTemporalMarker(.1, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(.3, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION3);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone four drop off
        backToDropOff3 = drive.trajectorySequenceBuilder(backToPickup3.end())
                .strafeTo(new Vector2d(-20, -6))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWLEFTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();

        // cone five pickup
        backToPickup4 = drive.trajectorySequenceBuilder(backToDropOff3.end())
                .strafeTo(new Vector2d(-55, -13))
                .addTemporalMarker(.1, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(.3, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION2);
                })
                .addTemporalMarker(2.1, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone five drop off
        backToDropOff4 = drive.trajectorySequenceBuilder(backToPickup4.end())
                .strafeTo(new Vector2d(-20, -6))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWLEFTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();


        // cone six pickup - fix the drift by moving to -6.75
        backToPickup5 = drive.trajectorySequenceBuilder(backToDropOff4.end())
                .strafeTo(new Vector2d(-55, -13))
                .addTemporalMarker(.1, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(.3, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION1);
                })
                .addTemporalMarker(2.1, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                })
                .build();

        // cone six drop off
        backToDropOff5 = drive.trajectorySequenceBuilder(backToPickup5.end())
                .strafeTo(new Vector2d(-20, -6))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    clawTiltServo.setPosition(robotState.CLAWLEFTTILT);
                })
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .build();

        // park
        backToPickup6_left = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 40;
                    }
                })
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 60;
                    }
                })
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .back(30)
                .build();

        backToPickup6_middle = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .back(12)
                .build();

        backToPickup6_right = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(0.4, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(0.8, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .forward(10)
                .build();
    }
}
