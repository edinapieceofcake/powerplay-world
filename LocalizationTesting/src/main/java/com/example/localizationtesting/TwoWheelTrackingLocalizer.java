package com.example.localizationtesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

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
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1000;
    public static double WHEEL_RADIUS = 1.35826772 / 2; // in
    public static double GEAR_RATIO = 22.0/30.0; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -0.5625; // X is the up and down direction
    public static double PARALLEL_Y = -4.6875; // Y is the strafe direction

    public static double PERPENDICULAR_X = 2.9375;
    public static double PERPENDICULAR_Y = -0.125;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis

    public static double X_MULTIPLIER = 0.95051; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.94529; // Multiplier in the Y direction

    List<Double> parallelEncoderPositions;
    List<Double> perpendicularEncoderPositions;
    List<Double> parallelEncoderVelocities;
    List<Double> perpendicularEncoderVelocities;

    List<Double> headingValues;
    List<Double> headingVelocityValues;

    public int currentPosition = 0;

    public TwoWheelTrackingLocalizer() {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, Math.toRadians(0)),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public TwoWheelTrackingLocalizer(double parallelX, double parallelY, double parallelHeading, double perpendicularX, double perpendicularY, double perpendicularHeading,
                                     List<Double> parallelEncoderPositions, List<Double> perpendicularEncoderPositions, List<Double> parallelEncoderVelocities,
                                     List<Double> perpendicularEncoderVelocities, List<Double> headingValues, List<Double> headingVelocityValues) {
        super(Arrays.asList(
                new Pose2d(parallelX, parallelY, Math.toRadians(parallelHeading)),
                new Pose2d(perpendicularX, perpendicularY, Math.toRadians(perpendicularHeading))
        ));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        this.headingValues = headingValues;
        this.headingVelocityValues = headingVelocityValues;
        this.parallelEncoderPositions = parallelEncoderPositions;
        this.parallelEncoderVelocities = parallelEncoderVelocities;
        this.perpendicularEncoderPositions = perpendicularEncoderPositions;
        this.perpendicularEncoderVelocities = perpendicularEncoderVelocities;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return headingValues.get(currentPosition);
    }

    @Override
    public Double getHeadingVelocity() {
        return headingVelocityValues.get(currentPosition);
    }

    @Override
    public List<Double> getWheelPositions() {

        return Arrays.asList(
                parallelEncoderPositions.get(currentPosition),
                perpendicularEncoderPositions.get(currentPosition)
        );
    }

    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                parallelEncoderVelocities.get(currentPosition),
                perpendicularEncoderVelocities.get(currentPosition)
        );
    }
}