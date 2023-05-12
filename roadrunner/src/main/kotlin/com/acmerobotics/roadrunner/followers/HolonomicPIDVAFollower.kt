package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * Traditional PID controller with feedforward velocity and acceleration components to follow a trajectory. More
 * specifically, the feedback is applied to the components of the robot's pose (x position, y position, and heading) to
 * determine the velocity correction. The feedforward components are instead applied at the wheel level.
 *
 * @param axialCoeffs PID coefficients for the robot axial controller (robot X)
 * @param lateralCoeffs PID coefficients for the robot lateral controller (robot Y)
 * @param headingCoeffs PID coefficients for the robot heading controller
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param timeout max time to wait for the error to be admissible
 * @param clock clock
 */
class HolonomicPIDVAFollower @JvmOverloads constructor(
    axialCoeffs: PIDCoefficients,
    lateralCoeffs: PIDCoefficients,
    headingCoeffs: PIDCoefficients,
    admissibleError: Pose2d = Pose2d(),
    timeout: Double = 0.0,
    clock: NanoClock = NanoClock.system()
) : TrajectoryFollower(admissibleError, timeout, clock) {
    private val axialController = PIDFController(axialCoeffs)
    private val lateralController = PIDFController(lateralCoeffs)
    private val headingController = PIDFController(headingCoeffs)

    override var lastError: Pose2d = Pose2d()

    init {
        headingController.setInputBounds(-Math.PI, Math.PI)
    }

    override fun followTrajectory(trajectory: Trajectory) {
        axialController.reset()
        lateralController.reset()
        headingController.reset()

        super.followTrajectory(trajectory)
    }

    override fun internalUpdate(currentPose: Pose2d, currentRobotVel: Pose2d?): DriveSignal {
        val t = elapsedTime()

        val targetPose = trajectory[t]
        val targetVel = trajectory.velocity(t)
        val targetAccel = trajectory.acceleration(t)

        val targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel)
        val targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel)

        val poseError = Kinematics.calculateRobotPoseError(targetPose, currentPose)

        println(String.format("UpdateTarget %f, %f, %f", targetPose.x, targetPose.y, targetPose.heading))
        println(String.format("UpdateCurrent %f, %f, %f", currentPose.x, currentPose.y, currentPose.heading))
        println(String.format("UpdateError %f, %f, %f", poseError.x, poseError.y, poseError.heading))

        // you can pass the error directly to PIDFController by setting setpoint = error and measurement = 0
        axialController.targetPosition = poseError.x
        lateralController.targetPosition = poseError.y
        headingController.targetPosition = poseError.heading

        axialController.targetVelocity = targetRobotVel.x
        lateralController.targetVelocity = targetRobotVel.y
        headingController.targetVelocity = targetRobotVel.heading

        // note: feedforward is processed at the wheel level
        val axialCorrection = axialController.update(0.0, currentRobotVel?.x)
        val lateralCorrection = lateralController.update(0.0, currentRobotVel?.y)
        val headingCorrection = headingController.update(0.0, currentRobotVel?.heading)

        val correction =  Pose2d(
            axialCorrection,
            lateralCorrection,
            headingCorrection
        )

        val correctedVelocity = targetRobotVel + correction

        println(String.format("UpdateCurrentVelocity %f, %f, %f", currentRobotVel?.x, currentRobotVel?.y, currentRobotVel?.heading))
        println(String.format("UpdateTargetVelocity %f, %f, %f", targetRobotVel.x, targetRobotVel.y, targetRobotVel.heading))
        println(String.format("UpdateCorrectionVelocity %f, %f, %f", correction.x, correction.y, correction.heading))
        println(String.format("UpdateCorrectedVelocity %f, %f, %f", correctedVelocity.x, correctedVelocity.y, correctedVelocity.heading))

        lastError = poseError

        return DriveSignal(correctedVelocity, targetRobotAccel)
    }
}
