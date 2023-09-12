package edu.edina.library.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import edu.edina.library.util.DriveSpeed;
import edu.edina.library.util.PoseStorage;
import edu.edina.library.util.RobotState;

public class MecanumDrive extends Subsystem{
    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private SampleMecanumDrive drive;
    private RobotState robotState;

    public MecanumDrive(HardwareMap map, RobotState robotState){
        //try {
            drive = new SampleMecanumDrive(map);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.setPoseEstimate(PoseStorage.currentPose);
            robotState.SpeedMultiplier = robotState.LowSpeedMultiplier;
            robotState.DriveSpeed = DriveSpeed.Low;
            this.robotState = robotState;
            robotState.DriveSuccessfullySetup = true;
        //} catch (Exception ex) {
           // robotState.DriveSuccessfullySetup = false;
        // }
    }

    @Override
    public void update() {
        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        // To change to Robot centric change leftSticks to ____
                        // To change to Robot centric change leftSticks to ____
                        -leftStickY,
                        -leftStickX,
                        -rightStickX
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();
    }

    public void setDriveProperties(double leftStickX, double leftStickY, double rightStickX, boolean dPadDown){
        if (dPadDown) {
            if (robotState.DriveSpeed == DriveSpeed.High) {
                robotState.SpeedMultiplier = robotState.LowSpeedMultiplier;
                robotState.DriveSpeed = DriveSpeed.Low;
            } else {
                robotState.DriveSpeed = DriveSpeed.High;
                robotState.SpeedMultiplier = robotState.HighSpeedMultiplier;
            }
        }

        this.leftStickX = ScaleMotorCube(leftStickX) * robotState.SpeedMultiplier;
        this.leftStickY = ScaleMotorCube(leftStickY) * robotState.SpeedMultiplier;
        this.rightStickX = ScaleMotorCube(rightStickX) * robotState.SpeedMultiplier;
    }

    public static double ScaleMotorCube(double joyStickPosition) {
        return (double) Math.pow(joyStickPosition, 3.0);
    }
}
