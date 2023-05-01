package org.firstinspires.ftc.teamcode.drive;

import static java.util.Collections.emptyList;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;
import java.util.List;

import javax.annotation.Nullable;

abstract class POCTwoTrackingWheelLocalizer {
    private Pose2d _poseEstimate = new Pose2d();

    public Pose2d getPoseEstimate() { return _poseEstimate; }
    public void setPoseEstimate(Pose2d value) {
        lastWheelPositions = emptyList();
        lastHeading = Double.NaN;
        this._poseEstimate = value;
    }

    private Pose2d poseVelocity = null;
    private List<Double> lastWheelPositions = new ArrayList<>();
    private Double lastHeading = Double.NaN;

    private DecompositionSolver forwardSolver;

    public POCTwoTrackingWheelLocalizer(List<Pose2d> wheelPoses) {
        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);
        for (int i = 0; i < wheelPoses.size(); i++) {
            Vector2d orientationVector = wheelPoses.get(i).headingVec();
            Vector2d positionVector = wheelPoses.get(i).vec();
            inverseMatrix.setEntry(i, 0, orientationVector.getX());
            inverseMatrix.setEntry(i, 1, orientationVector.getY());
            inverseMatrix.setEntry(
                    i,
                    2,
                    positionVector.getX() * orientationVector.getY() - positionVector.getY() * orientationVector.getX()
            );
        }
        inverseMatrix.setEntry(2, 2, 1.0);

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
    }

    private Pose2d calculatePoseDelta(List<Double> wheelDeltas, Double headingDelta) {
        RealMatrix matrix = MatrixUtils.createRealMatrix(3, 1);
        matrix.setEntry(0, 0, wheelDeltas.get(0));
        matrix.setEntry(1, 0, wheelDeltas.get(1));
        matrix.setEntry(2, 0, headingDelta);
        RealMatrix rawPoseDelta = forwardSolver.solve(matrix).transpose();

        return new Pose2d(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(0, 1),
                rawPoseDelta.getEntry(0, 2)
        );
    }

    public void update() {
        List<Double> wheelPositions = getWheelPositions();
        Double heading = getHeading();
        if (!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (int i = 0; i < lastWheelPositions.size(); i++) {
                wheelDeltas.add(lastWheelPositions.get(i) - wheelPositions.get(i));
            }
            Double headingDelta = Angle.normDelta(heading - lastHeading);
            Pose2d robotPoseDelta = calculatePoseDelta(wheelDeltas, headingDelta);
            _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, robotPoseDelta);
        }

        List<Double> wheelVelocities = getWheelVelocities();
        Double headingVelocity = getHeadingVelocity();
        if (wheelVelocities != null && headingVelocity != null) {
            poseVelocity = calculatePoseDelta(wheelVelocities, headingVelocity);
        }

        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }

    abstract List<Double> getWheelPositions();

    /**
     * Returns the velocities of the tracking wheels in the desired distance units (not encoder counts!)
     */
    abstract List<Double> getWheelVelocities();

    /**
     * Returns the heading of the robot (usually from a gyroscope or IMU).
     */
    abstract double getHeading();

    /**
     * Returns the heading of the robot (usually from a gyroscope or IMU).
     */
    abstract Double getHeadingVelocity();
}
