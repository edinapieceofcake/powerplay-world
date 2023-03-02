package edu.edina.opmodes.auto;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(group = "Left")
@Config
@Disabled
public class LeftSideHigh extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double dropTime = .5;
    double pickupTime = .5;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int detectionId = 6; // middle

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public static double PICKUPY = -8.5;

    private DcMotorEx liftMotor;
    private RobotState robotState = new RobotState();
    private Servo armServo;
    private Servo clawServo;
    private ColorSensor backColor;
    private DistanceSensor frontDistance;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "leftCamera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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

        telemetry.setMsTransmissionInterval(50);

        Servo leftServo = hardwareMap.get(Servo.class, "leftPodServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightPodServo");
        Servo centerServo = hardwareMap.get(Servo.class, "centerPodServo");
        leftServo.setPosition(robotState.SERVODOWNPOSITION);
        rightServo.setPosition(robotState.SERVODOWNPOSITION);
        centerServo.setPosition(robotState.SERVODOWNPOSITION);

        rightServo.setPosition(1);
        centerServo.setPosition(1);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        backColor = hardwareMap.get(ColorSensor.class, "backColor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        final float[] hsvValues = new float[3];

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setPower(1);
        robotState.FutureTargetPosition = 0;
        liftMotor.setTargetPosition(robotState.FutureTargetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;

        armServo.setPosition(robotState.ARMFRONTPOSITION);
        robotState.ArmServoPosition = ArmServoPosition.Front;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-33, -65, Math.toRadians(0)));

        // cone one drop off
        TrajectorySequence start = drive.trajectorySequenceBuilder(new Pose2d(-33, -65, Math.toRadians(0)))
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
                .strafeTo(new Vector2d(-30, -20))
                .build();

        // cone two pickup
        TrajectorySequence backToPickup1 = drive.trajectorySequenceBuilder(start.end())
                .strafeLeft(13)
                .back(20)
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(1, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION5);
                })
                .build();

        TrajectorySequence strafe = drive.trajectorySequenceBuilder(backToPickup1.end())
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 20;
                    }
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 20;
                    }
                })
                .strafeRight(5).build();

        // cone two dropoff
        TrajectorySequence backToDropOff1 = drive.trajectorySequenceBuilder(new Pose2d(-60, -8, Math.toRadians(0)))
                .resetConstraints()
                .strafeTo(new Vector2d(-25, -4))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 10);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    } )
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone three pickup
        TrajectorySequence backToPickup2 = drive.trajectorySequenceBuilder(backToDropOff1.end())
                .strafeTo(new Vector2d(-59.5, -8))
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(1, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION4);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone three dropoff
        TrajectorySequence backToDropOff2 = drive.trajectorySequenceBuilder(backToPickup2.end())
                .strafeTo(new Vector2d(-25.5, -4))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 10);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone four pickup
        TrajectorySequence backToPickup3 = drive.trajectorySequenceBuilder(backToDropOff2.end())
                .strafeTo(new Vector2d(-59.5, -8))
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(1, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION3);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone four drop off
        TrajectorySequence backToDropOff3 = drive.trajectorySequenceBuilder(backToPickup3.end())
                .strafeTo(new Vector2d(-25.5, -4))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 10);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone five pickup
        TrajectorySequence backToPickup4 = drive.trajectorySequenceBuilder(backToDropOff3.end())
                .strafeTo(new Vector2d(-59.5, -8))
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(.7, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION2);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone five drop off
        TrajectorySequence backToDropOff4 = drive.trajectorySequenceBuilder(backToPickup4.end())
                .strafeTo(new Vector2d(-25.5, -4))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 10);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();


        // cone six pickup - fix the drift by moving to -6.75
        TrajectorySequence backToPickup5 = drive.trajectorySequenceBuilder(backToDropOff4.end())
                .strafeTo(new Vector2d(-59.5, -8))
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(.7, () -> {
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION1);
                })
                .addTemporalMarker(1.9, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone six drop off
        TrajectorySequence backToDropOff5 = drive.trajectorySequenceBuilder(backToPickup5.end())
                .strafeTo(new Vector2d(-25.5, -4))
                .addTemporalMarker(.1, () -> {
                    liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITIONHIGH - 40);
                })
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2.0, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // park
        TrajectorySequence backToPickup6_left = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 40;
                    }
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 45;
                    }
                })
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                    clawServo.setPosition(robotState.CLAWMIDDLEPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Middle;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .back(39)
                .build();

        TrajectorySequence backToPickup6_middle = drive.trajectorySequenceBuilder(backToDropOff5.end())
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

        TrajectorySequence backToPickup6_right = drive.trajectorySequenceBuilder(backToDropOff5.end())
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

        while (!isStarted() && !isStopRequested())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("Make sure claw is in the front and high camera is facing field.", "");
                telemetry.addData("Cone should always be on side with medium pole", "");
                telemetry.addData("If the distance number is huge, turn the power off and on. Wait five seconds before turning back on after turning off.", "");

                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
                telemetry.addData("Distance", frontDistance.getDistance(DistanceUnit.INCH));

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        detectionId = detection.id;
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    }
                }

                telemetry.update();
            }

            sleep(20);
        }

        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });

        if (opModeIsActive()) {
            this.resetRuntime();
            liftMotor.setPower(1);
            drive.followTrajectorySequence(start);

            drive.followTrajectorySequence(backToPickup1);

            drive.followTrajectorySequenceAsync(strafe);
            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                Color.RGBToHSV(backColor.red() * 8, backColor.green() * 8, backColor.blue() * 8, hsvValues);
                if ((hsvValues[0] < 100) || (hsvValues[0] > 180)){
                    drive.breakFollowing();
                    drive.setDrivePower(new Pose2d());
                    break;
                }

                drive.update();
                idle();
            }

            double lastDistanceToTravel = frontDistance.getDistance(DistanceUnit.INCH) ;

            TrajectorySequence back = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setVelConstraint(new TrajectoryVelocityConstraint() {
                        @Override
                        public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                            return 25;
                        }
                    })
                    .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                        @Override
                        public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                            return 20;
                        }
                    })
                    .back(lastDistanceToTravel)
                    .build();

            drive.followTrajectorySequence(back);

            clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Closed;

            sleep(200);

            drive.setPoseEstimate(new Pose2d(-60, -8, Math.toRadians(0)));

            drive.followTrajectorySequence(backToDropOff1);

            drive.followTrajectorySequence(backToPickup2);

            drive.followTrajectorySequence(backToDropOff2);

            drive.followTrajectorySequence(backToPickup3);

            drive.followTrajectorySequence(backToDropOff3);

            drive.followTrajectorySequence(backToPickup4);

            drive.followTrajectorySequence(backToDropOff4);

            drive.followTrajectorySequence(backToPickup5);

            drive.followTrajectorySequence(backToDropOff5);

            if (detectionId == 3) {
                drive.followTrajectorySequence(backToPickup6_left);
            } else if (detectionId == 6) {
                drive.followTrajectorySequence(backToPickup6_middle);
            } else {
                drive.followTrajectorySequence(backToPickup6_right);
            }

            while (opModeIsActive()) {
                double currentDistanceToTravel = frontDistance.getDistance(DistanceUnit.INCH);
                telemetry.addData("lastDistanceToTravel", lastDistanceToTravel);
                telemetry.addData("currentDistanceToTravel", currentDistanceToTravel);
                telemetry.update();
                sleep(20);
            }
        }
    }
}
