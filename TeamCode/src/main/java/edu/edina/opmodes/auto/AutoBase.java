package edu.edina.opmodes.auto;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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


@Config
public class AutoBase extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;
    protected SampleMecanumDrive drive;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public static double PICKUPY = -8.5;

    protected DcMotorEx liftMotor;
    protected RobotState robotState = new RobotState();
    protected Servo armServo;
    protected Servo clawServo;
    protected Servo clawTiltServo;

    protected DcMotorEx leftFront;
    protected DcMotorEx leftRear;
    protected DcMotorEx rightRear;
    protected DcMotorEx rightFront;
    protected DistanceSensor distanceSensor;

    protected TrajectorySequence start;
    protected TrajectorySequence backToPickup1;
    protected TrajectorySequence backToPickup2;
    protected TrajectorySequence backToPickup3;
    protected TrajectorySequence backToPickup4;
    protected TrajectorySequence backToPickup5;
    protected TrajectorySequence backToDropOff1;
    protected TrajectorySequence backToDropOff2;
    protected TrajectorySequence backToDropOff3;
    protected TrajectorySequence backToDropOff4;
    protected TrajectorySequence backToDropOff5;
    protected TrajectorySequence backToPickup6_left;
    protected TrajectorySequence backToPickup6_middle;
    protected TrajectorySequence backToPickup6_right;

    protected int detectionId = 9;

    protected boolean shouldClawBeInTheFront() {
        return true;
    }

    protected String getCameraName() {
        assert false;
        return "";
    }

    protected void initHardware() {
        Servo leftServo = hardwareMap.get(Servo.class, "leftPodServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightPodServo");
        Servo centerServo = hardwareMap.get(Servo.class, "centerPodServo");
        leftServo.setPosition(robotState.SERVODOWNPOSITION);
        rightServo.setPosition(robotState.SERVODOWNPOSITION);
        centerServo.setPosition(robotState.SERVODOWNPOSITION);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawTiltServo = hardwareMap.get(Servo.class, "clawTiltServo");

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistance");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotState.FutureTargetPosition = 0;
        liftMotor.setTargetPosition(robotState.FutureTargetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawTiltServo.setPosition(robotState.CLAWCENTERTILT);

        // wait for the claw to center before we close during init
        sleep(500);

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;

        if (shouldClawBeInTheFront()) {
            armServo.setPosition(robotState.ARMFRONTPOSITION);
            robotState.ArmServoPosition = ArmServoPosition.Front;
        } else {
            armServo.setPosition(robotState.ARMBACKPOSITION);
            robotState.ArmServoPosition = ArmServoPosition.Back;
        }

        drive = new SampleMecanumDrive(hardwareMap);
    }

    protected void initPaths() {
        assert false;
    }

    protected void addAdditionalTelemetry(Telemetry telemetry) {}

    protected Pose2d getStartPose() { return null; }
    protected void setDetectionId() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, getCameraName()), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        drive = new SampleMecanumDrive(hardwareMap);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted() && !isStopRequested()) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                addAdditionalTelemetry(telemetry);

                telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        detectionId = detection.id;
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    }
                }

                telemetry.update();
            }

            sleep(20);
        }

        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            public void onClose() {

            }
        });
    }

    protected void checkToMoveBackwards() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        double difference = distance - 35;

        if (difference > 0) {
            double travelDistance = difference / 301.0 * 384.5;
            long futurePosition = leftFront.getCurrentPosition() - (long)travelDistance;
            long futureTime = System.currentTimeMillis() + 500;

            leftFront.setPower(-.1);
            leftRear.setPower(-.1);
            rightRear.setPower(-.1);
            rightFront.setPower(-.1);

            while (opModeIsActive() && (leftFront.getCurrentPosition() > futurePosition) && (System.currentTimeMillis() < futureTime)) {
                idle();
            }

            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            rightFront.setPower(0);
        }
    }

    protected void checkToMoveForward() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        double difference = distance - 35;

        if (difference > 0) {
            double travelDistance = difference / 301.0 * 384.5;
            long futurePosition = leftFront.getCurrentPosition() + (long)travelDistance;
            long futureTime = System.currentTimeMillis() + 500;

            leftFront.setPower(.1);
            leftRear.setPower(.1);
            rightRear.setPower(.1);
            rightFront.setPower(.1);

            while (opModeIsActive() && (leftFront.getCurrentPosition() < futurePosition) && (System.currentTimeMillis() < futureTime)) {
                idle();
            }

            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            rightFront.setPower(0);
        }
    }

    protected void runPaths() {
        drive.setPoseEstimate(getStartPose());

        drive.followTrajectorySequence(start);

        drive.followTrajectorySequence(backToPickup1);

        drive.followTrajectorySequence(backToDropOff1);

        drive.followTrajectorySequence(backToPickup2);

        drive.followTrajectorySequence(backToDropOff2);

        drive.followTrajectorySequence(backToPickup3);

        drive.followTrajectorySequence(backToDropOff3);

        drive.followTrajectorySequence(backToPickup4);

        drive.followTrajectorySequence(backToDropOff4);

        drive.followTrajectorySequence(backToPickup5);

        drive.followTrajectorySequence(backToDropOff5);
    }

    protected void runPark() {
        if (detectionId == 3) {
            drive.followTrajectorySequence(backToPickup6_left);
        } else if (detectionId == 6) {
            drive.followTrajectorySequence(backToPickup6_middle);
        } else {
            drive.followTrajectorySequence(backToPickup6_right);
        }
    }

    @Override
    public void runOpMode() {

        initHardware();

        initPaths();

        setDetectionId();
        
        if (opModeIsActive()) {
            liftMotor.setPower(1);

            runPaths();

            runPark();
        }
    }
}