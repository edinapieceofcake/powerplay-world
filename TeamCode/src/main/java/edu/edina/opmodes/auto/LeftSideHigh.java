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

    protected Pose2d getStartPose() {
        return new Pose2d(-33, -65, Math.toRadians(0));
    }

    @Override
    protected void initPaths() {
        // cone one drop off
        start = drive.trajectorySequenceBuilder(getStartPose())
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.POLEPOSITIONLOW);
                })
                .addTemporalMarker(1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM - 10);
                    clawTiltServo.setPosition(robotState.CLAWRIGHTTILT);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                })
                .strafeTo(new Vector2d(-30, -21))
                .build();

        // cone two pickup
        backToPickup1 = drive.trajectorySequenceBuilder(start.end())
                .strafeLeft(11)
                .strafeTo(new Vector2d(-54, -14))
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(.7, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION5);
                })
                .build();

        // cone two dropoff
        backToDropOff1 = drive.trajectorySequenceBuilder(backToPickup1.end())
                .strafeTo(new Vector2d(-18, -11.25))
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(.4, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 10);
                })
                .addTemporalMarker(1.3, () -> {
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
                .strafeTo(new Vector2d(-54, -14.5))
                .addTemporalMarker(.1, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(.3, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION4);
                })
                .build();

        // cone three dropoff
        backToDropOff2 = drive.trajectorySequenceBuilder(backToPickup2.end())
                .strafeTo(new Vector2d(-19, -11.25))
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(.4, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 10);
                })
                .addTemporalMarker(1.3, () -> {
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
                .strafeTo(new Vector2d(-55, -16))
                .addTemporalMarker(.1, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(.3, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION3);
                })
                .build();

        // cone four drop off
        backToDropOff3 = drive.trajectorySequenceBuilder(backToPickup3.end())
                .strafeTo(new Vector2d(-20, -13))
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(.4, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 10);
                })
                .addTemporalMarker(1.3, () -> {
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
                .strafeTo(new Vector2d(-55, -16))
                .addTemporalMarker(.1, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(.3, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION2);
                })
                .build();

        // cone five drop off
        backToDropOff4 = drive.trajectorySequenceBuilder(backToPickup4.end())
                .strafeTo(new Vector2d(-20, -13))
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(.4, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 10);
                })
                .addTemporalMarker(1.3, () -> {
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
                .strafeTo(new Vector2d(-54, -16))
                .addTemporalMarker(.1, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(.3, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION1);
                })
                .build();

        // cone six drop off
        backToDropOff5 = drive.trajectorySequenceBuilder(backToPickup5.end())
                .strafeTo(new Vector2d(-20, -13))
                .addTemporalMarker(0, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONMEDIUM);
                })
                .addTemporalMarker(.4, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 30);
                })
                .addTemporalMarker(1.3, () -> {
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
                .back(37)
                .build();

        backToPickup6_middle = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .back(5)
                .build();

        backToPickup6_right = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .forward(10)
                .build();
    }

    @Override
    protected void runPaths() {

        drive.setPoseEstimate(getStartPose());

        drive.followTrajectorySequence(start);

        drive.followTrajectorySequence(backToPickup1);

        checkToMoveBackwards();

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(300);

        drive.followTrajectorySequence(backToDropOff1);

        drive.followTrajectorySequence(backToPickup2);

        checkToMoveBackwards();

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(300);

        drive.followTrajectorySequence(backToDropOff2);

        drive.followTrajectorySequence(backToPickup3);

        checkToMoveBackwards();

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(300);

        drive.followTrajectorySequence(backToDropOff3);

        drive.followTrajectorySequence(backToPickup4);

        checkToMoveBackwards();

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(300);

        drive.followTrajectorySequence(backToDropOff4);

        drive.followTrajectorySequence(backToPickup5);

        checkToMoveBackwards();

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;
        sleep(300);

        drive.followTrajectorySequence(backToDropOff5);
    }
}
