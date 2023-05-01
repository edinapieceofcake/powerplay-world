package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class POCTwoWheelTrackingLocalizer extends POCTwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1000;
    public static double WHEEL_RADIUS = 1.35826772 / 2; // in
    public static double GEAR_RATIO = 22.0/30.0; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -0.5625; // X is the up and down direction
    public static double PARALLEL_Y = -4.6875; // Y is the strafe direction

    public static double PERPENDICULAR_X = 2.9875;
    public static double PERPENDICULAR_Y = -0.125;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    public static double X_MULTIPLIER = 0.95051; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.94529; // Multiplier in the Y direction

    private SampleMecanumDrive drive;
    private List<Integer> lastEncPositions, lastEncVels;

    public POCTwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;
        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "centerEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public POCTwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels,
                                        double parallelX, double parallelY, double parallelHeading, double perpendicularX, double perpendicularY, double perpendicularHeading) {
        super(Arrays.asList(
                new Pose2d(parallelX, parallelY, Math.toRadians(parallelHeading)),
                new Pose2d(perpendicularX, perpendicularY, Math.toRadians(perpendicularHeading))
        ));

        this.drive = drive;
        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "centerEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = parallelEncoder.getCurrentPosition();
        int rightPos = parallelEncoder.getCurrentPosition();
        int frontPos = perpendicularEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        int leftPos = parallelEncoder.getCurrentPosition();
        int rightPos = parallelEncoder.getCurrentPosition();
        int frontPos = perpendicularEncoder.getCurrentPosition();

        lastEncVels.clear();
        lastEncVels.add(leftPos);
        lastEncVels.add(rightPos);
        lastEncVels.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getRawVelocity() * X_MULTIPLIER),
                encoderTicksToInches(perpendicularEncoder.getRawVelocity() * Y_MULTIPLIER)
        );
    }
}