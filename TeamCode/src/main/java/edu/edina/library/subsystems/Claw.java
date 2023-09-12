package edu.edina.library.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.library.util.RobotState;
import edu.edina.library.util.ClawRotation;

public class Claw extends edu.edina.library.subsystems.Subsystem {
    private Servo clawTiltServo;
    private RobotState robotState;

    public Claw(HardwareMap map, RobotState robotState) {
        clawTiltServo = map.get(Servo.class, "clawTiltServo");
        this.robotState = robotState;
        robotState.ClawRotation = ClawRotation.Pickup;
    }

    public void setClawProperties(boolean bumper_left, boolean bumper_right){
        if (bumper_left){
            robotState.ClawRotation = ClawRotation.Dropoff;
        }
        else if (bumper_right){
            robotState.ClawRotation = ClawRotation.Pickup;
        }
    }


    @Override
    public void update() {
        if (robotState.LiftMotorLocation < -425) {
            robotState.ClawRotation = ClawRotation.Dropoff;
        } else {
            robotState.ClawRotation = ClawRotation.Pickup;
        }

        switch (robotState.ClawRotation){
            case Pickup:
                clawTiltServo.setPosition(robotState.RI30HCLAWPICKUP);
                break;
            case Dropoff:
                clawTiltServo.setPosition(robotState.RI30HCLAWDROPOFF);
                break;
        }

    }
}
