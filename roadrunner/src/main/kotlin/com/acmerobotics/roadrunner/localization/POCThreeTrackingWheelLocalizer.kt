package com.acmerobotics.roadrunner.localization

import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.util.Angle
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.DecompositionSolver
import org.apache.commons.math3.linear.LUDecomposition
import org.apache.commons.math3.linear.MatrixUtils

/**
 * Localizer based on three unpowered tracking omni wheels.
 *
 * @param wheelPoses wheel poses relative to the center of the robot (positive X points forward on the robot)
 * @param drive wheel poses relative to the center of the robot (positive X points forward on the robot)
 */
abstract class POCThreeTrackingWheelLocalizer(
    wheelPoses: List<Pose2d>,
    drive: Drive
) : Localizer {
    private var _poseEstimate = Pose2d()
    private var drive: Drive
    private var distanceBetweenParallelEncoders: Double
    private var xDistanceOfFrontEncoderFromCenter: Double
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
    private var lastWheelPositions = emptyList<Double>()

    private val forwardSolver: DecompositionSolver

    init {
        require(wheelPoses.size == 3) { "3 wheel positions must be provided" }

        this.drive = drive;
        val inverseMatrix = Array2DRowRealMatrix(3, 3)
        for (i in 0..2) {
            val orientationVector = wheelPoses[i].headingVec()
            val positionVector = wheelPoses[i].vec()
            inverseMatrix.setEntry(i, 0, orientationVector.x)
            inverseMatrix.setEntry(i, 1, orientationVector.y)
            inverseMatrix.setEntry(
                i,
                2,
                positionVector.x * orientationVector.y - positionVector.y * orientationVector.x
            )
        }

        forwardSolver = LUDecomposition(inverseMatrix).solver

        require(forwardSolver.isNonSingular) { "The specified configuration cannot support full localization" }

        distanceBetweenParallelEncoders = wheelPoses.get(0).y * 2.0
        xDistanceOfFrontEncoderFromCenter = wheelPoses.get(2).x
    }

    private fun calculatePoseDelta(wheelDeltas: List<Double>): Pose2d {
        val rawPoseDelta = forwardSolver.solve(
            MatrixUtils.createRealMatrix(
                arrayOf(wheelDeltas.toDoubleArray())
            ).transpose()
        )
        return Pose2d(
            rawPoseDelta.getEntry(0, 0),
            rawPoseDelta.getEntry(1, 0),
            rawPoseDelta.getEntry(2, 0)
        )
    }

    /*
public void odometry() {
https://www.youtube.com/watch?v=Av9ZMjS--gY
    oldRightPosition = currentRightPosition;
    oldLeftPosition = currentLeftPosition;
    oldAuxPosition = currentAuxPosition;

    currentRightPosition = -encoderRight.getCurrentPosition();
    currentLeftPosition = encoderLeft.getCurrentPosition();
    currentAuxPosition = encoderAux.getCurrentPosition();

    int dn1 = currentLeftPosition  - oldLeftPosition;
    int dn2 = currentRightPosition - oldRightPosition;
    int dn3 = currentAuxPosition - oldAuxPosition;

    // the robot has moved and turned a tiny bit between two measurements:
    double dtheta = cm_per_tick * ((dn2-dn1) / (LENGTH));
    double dx = cm_per_tick * ((dn1+dn2) / 2.0);
    double dy = cm_per_tick * (dn3 + ((dn2-dn1) / 2.0));

    telemetrydx = dx;
    telemetrydy = dy;
    telemetrydh = dtheta;

    // small movement of the robot gets added to the field coordinate system:
    pos.h += dtheta / 2;
    pos.x += dx * Math.cos(pos.h) - dy * Math.sin(pos.h);
    pos.y += dx * Math.sin(pos.h) + dy * Math.cos(pos.h);
    pos.h += dtheta / 2;
    pos.h = normDiff(pos.h);
}
     */
    /*
https://github.com/mnocito/encoderlocalization/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BlackBoxBot.java
        public void updatePosition() {
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDistance - deltaRightDistance) / wheelDistanceApart;
        resetTicks();
    }
     */
    // https://github.com/FTCLib/FTCLib/blob/v2.0.1/core/src/main/java/com/arcrobotics/ftclib/kinematics/HolonomicOdometry.java
    private fun calculatePoseDelta2(wheelDeltas: List<Double>) : Pose2d {
        val dn1: Double = wheelDeltas.get(0)
        val dn2: Double = wheelDeltas.get(1)
        val dn3: Double = wheelDeltas.get(2)

        val dtheta: Double = ((dn2 - dn1) / distanceBetweenParallelEncoders)
        val dx: Double = ((dn1 + dn2) / 2.0)
        val dy: Double = (dn3 + (dn2 - dn1) * xDistanceOfFrontEncoderFromCenter / distanceBetweenParallelEncoders)

        // small movement of the robot gets added to the field coordinate system:
        val theta: Double = _poseEstimate.heading + dtheta / 2

        return Pose2d(
            _poseEstimate.x + (dx * Math.cos(theta) - dy * Math.sin(theta)),
            _poseEstimate.y + (dx * Math.sin(theta) + dy * Math.cos(theta)),
            Angle.norm(_poseEstimate.heading + dtheta)
        )
    }

    override fun update() {
        val wheelPositions = getWheelPositions()
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
            _poseEstimate = calculatePoseDelta2(wheelDeltas)
            //_poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, deltaPos)
            System.out.println(String.format("WheelPose-POC %f %f %f %f", _poseEstimate.x, _poseEstimate.y, Math.toDegrees(_poseEstimate.heading), Math.toDegrees(getHeading())))
        }

        val wheelVelocities = getWheelVelocities()
        if (wheelVelocities != null) {
            poseVelocity = calculatePoseDelta(wheelVelocities)
        }

        lastWheelPositions = wheelPositions
    }

    /**
     * Returns the positions of the tracking wheels in the desired distance units (not encoder counts!)
     */
    abstract fun getWheelPositions(): List<Double>

    abstract fun getHeading(): Double

    /**
     * Returns the velocities of the tracking wheels in the desired distance units (not encoder counts!)
     */
    open fun getWheelVelocities(): List<Double>? = null
}
