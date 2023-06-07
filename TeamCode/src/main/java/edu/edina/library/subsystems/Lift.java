package edu.edina.library.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.sql.Time;

import edu.edina.library.util.ClawRotation;
import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.PoleLocation;
import edu.edina.library.util.RobotHardware;
import edu.edina.library.util.RobotState;

public class Lift extends edu.edina.library.subsystems.Subsystem {

    private RobotState robotState;
    private RobotHardware robotHardware;
    private boolean runningToPosition;
    private boolean atZeroPosition;
    private int targetPosition = 0;
    private long clawOpenStartedTime = 0;
    private boolean liftMotorReset = false;
    private boolean clawOpen = false;
    private long lastUpdateTime = 0;

    public Lift(RobotState robotState, RobotHardware robotHardware) {
        try {
            this.robotHardware = robotHardware;

            robotHardware.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotHardware.liftMotor.setTargetPosition(robotState.FutureTargetPosition);
            robotHardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.liftMotor.setPower(robotState.LiftUpSpeed);
            robotState.FutureTargetPosition = 0;
            robotHardware.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robotState.TargetPoleLocation = PoleLocation.None;
            robotState.LiftSuccessfullySetup = true;
            lastUpdateTime = System.currentTimeMillis();
        } catch (Exception ex) {
            robotState.LiftSuccessfullySetup = false;
        }

        this.robotState = robotState;

    }

    public void start(){
        robotHardware.clawServo.setPosition(robotState.CLAWOPENPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Open;

        robotHardware.armServo.setPosition(robotState.ARMFRONTPOSITION);
        robotState.ArmServoPosition = ArmServoPosition.Front;

        robotHardware.clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
        robotState.ClawRotation = ClawRotation.Center;
    }

    @Override
    public void update() {
        if (robotState.TargetPoleLocation != PoleLocation.None) {
            if (robotState.TargetPoleLocation == PoleLocation.Return) {
                if (!runningToPosition) {
                    robotHardware.clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotHardware.clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                    robotState.ClawRotation = ClawRotation.Center;
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawOpenStartedTime = System.currentTimeMillis();
                    runningToPosition = true;
                    clawOpen = false;
                } else if (!clawOpen) {
                    if ((System.currentTimeMillis() > (clawOpenStartedTime + robotState.CLAWOPENWAITTIME)) && (Math.round(robotHardware.clawServo.getPosition() * 100) == robotState.CLAWOPENPOSITION100)) {
                        robotHardware.liftMotor.setTargetPosition(robotState.LIFTRETURNHEiGHT);
                        robotHardware.liftMotor.setPower(robotState.LiftDownSpeed);
                        clawOpen = true;
                        atZeroPosition = false;
                    }
                } else if (!atZeroPosition) {
                    robotState.LiftDiff = Math.abs(robotHardware.liftMotor.getCurrentPosition());
                    robotState.FutureTargetPosition = robotHardware.liftMotor.getTargetPosition();

                    if (robotState.LiftDiff < 10) {
                        resetState();
                    }
                }
            } else {
                if (!runningToPosition) {
                    if (robotState.TargetPoleLocation == PoleLocation.Low) {
                        targetPosition = robotState.POLEPOSITIONLOW;
                    } else if (robotState.TargetPoleLocation == PoleLocation.Medium) {
                        targetPosition = robotState.POLEPOSITIONMIDDLE;
                    } else if (robotState.TargetPoleLocation == PoleLocation.High) {
                        targetPosition = robotState.POLEPOSITIONHIGH;
                    }

                    robotHardware.clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                    robotHardware.liftMotor.setTargetPosition(targetPosition);
                    robotHardware.liftMotor.setPower(robotState.LiftUpSpeed);
                    runningToPosition = true;
                } else {
                    robotState.FutureTargetPosition = robotHardware.liftMotor.getTargetPosition();

                    if (robotState.ClawServoPosition == ClawServoPosition.Open) {
                        robotHardware.clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    } else if (robotState.ClawServoPosition == ClawServoPosition.Closed) {
                        robotHardware.clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    }

                    if (robotState.ArmServoPosition == ArmServoPosition.Front) {
                        robotHardware.armServo.setPosition(robotState.ARMFRONTPOSITION);
                    } else if (robotState.ArmServoPosition == ArmServoPosition.Side) {
                        robotHardware.armServo.setPosition(robotState.ARMSIDEPOSITION);
                    } else if (robotState.ArmServoPosition == ArmServoPosition.Back) {
                        robotHardware.armServo.setPosition(robotState.ARMBACKPOSITION);
                    }
                }
            }
        } else {
            if (!robotHardware.liftSwitch.getState()) {
                if (!liftMotorReset) {
                    robotHardware.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robotHardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robotHardware.liftMotor.setPower(robotState.LiftUpSpeed);
                    robotHardware.liftMotor.setTargetPosition(0);
                    robotState.FutureTargetPosition = 0;
                    liftMotorReset = true;
                }
            } else {
                liftMotorReset = false;
            }

            robotHardware.liftMotor.setTargetPosition(robotState.FutureTargetPosition);
        }

        if (robotState.ClawServoPosition == ClawServoPosition.Open) {
            robotHardware.clawServo.setPosition(robotState.CLAWOPENPOSITION);
        } else if (robotState.ClawServoPosition == ClawServoPosition.Closed) {
            robotHardware.clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        }

        if (robotState.ArmServoPosition == ArmServoPosition.Front) {
            robotHardware.armServo.setPosition(robotState.ARMFRONTPOSITION);
        } else if (robotState.ArmServoPosition == ArmServoPosition.Side) {
            robotHardware.armServo.setPosition(robotState.ARMSIDEPOSITION);
        } else if (robotState.ArmServoPosition == ArmServoPosition.Back) {
            robotHardware.armServo.setPosition(robotState.ARMBACKPOSITION);
        }

        robotState.LiftMotorLocation = robotHardware.liftMotor.getCurrentPosition();
        robotState.ClawPosition = Math.round(robotHardware.clawServo.getPosition() * 100);
        robotState.ArmPosition = Math.round(robotHardware.armServo.getPosition() * 100);
        robotState.LiftSwitch = robotHardware.liftSwitch.getState();
        robotState.LiftMotorReset = liftMotorReset;
    }

    public void setLiftProperties(double liftDown, double liftUp, boolean armFront, boolean armSide, boolean armBack,
                                  boolean clawOpen, boolean clawClosed, boolean lowPole, boolean mediumPole,
                                  boolean highPole, boolean returnPosition, long liftDelatTimeInMS) {

        if (robotState.TargetPoleLocation != PoleLocation.None) {
            if ((liftUp != 0) || (liftDown != 0)) {
                resetState();
            }
        }

        if (System.currentTimeMillis() > (lastUpdateTime + liftDelatTimeInMS)) {
            lastUpdateTime = System.currentTimeMillis();
            if (liftDown != 0) {
                robotState.FutureTargetPosition += 15;
                robotHardware.liftMotor.setPower(robotState.LiftDownSpeed);
            } else if (liftUp != 0) {
                robotState.FutureTargetPosition += -15;
                robotHardware.liftMotor.setPower(robotState.LiftUpSpeed);
            }
        }

        if (armFront) {
            robotState.ArmServoPosition = ArmServoPosition.Front;
        } else if (armSide) {
            robotState.ArmServoPosition = ArmServoPosition.Side;
        } else if (armBack) {
            robotState.ArmServoPosition = ArmServoPosition.Back;
        }

        if (clawOpen) {
            robotState.ClawServoPosition = ClawServoPosition.Open;
        } else if (clawClosed) {
            robotState.ClawServoPosition = ClawServoPosition.Closed;
        }

        if (lowPole) {
            runningToPosition = false;
            robotState.TargetPoleLocation = PoleLocation.Low;
        } else if (mediumPole) {
            runningToPosition = false;
            robotState.TargetPoleLocation = PoleLocation.Medium;
        } else if (highPole) {
            runningToPosition = false;
            robotState.TargetPoleLocation = PoleLocation.High;
        } else if (returnPosition) {
            runningToPosition = false;
            robotState.TargetPoleLocation = PoleLocation.Return;
        }
    }

    private void resetState() {
        runningToPosition = false;
        atZeroPosition = false;
        robotState.TargetPoleLocation = PoleLocation.None;
        robotState.FutureTargetPosition = robotHardware.liftMotor.getTargetPosition();
        clawOpenStartedTime = 0;
    }
}
