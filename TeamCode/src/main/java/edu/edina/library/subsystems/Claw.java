package edu.edina.library.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import edu.edina.library.util.RobotState;
import edu.edina.library.util.ClawRotation;

public class Claw extends edu.edina.library.subsystems.Subsystem {
    private Servo clawTiltServo;
    private RobotState robotState;

    public Claw(HardwareMap map, RobotState robotState) {
        clawTiltServo = map.get(Servo.class, "clawTiltServo");
        this.robotState = robotState;
        robotState.ClawRotation = ClawRotation.Center;
    }

    public void setClawProperties(boolean dpad_left, boolean dpad_up, boolean dpad_right,boolean x, boolean y, boolean b) {
        if (dpad_left) {
            robotState.ClawRotation = ClawRotation.LeftDropoff;
        } else if(dpad_up){
            robotState.ClawRotation = ClawRotation.Center;
        } else if(dpad_right){
            robotState.ClawRotation = ClawRotation.RightDropoff;
        } else if (x){
            robotState.ClawRotation = ClawRotation.LeftPickup;
        } else if (y){
            robotState.ClawRotation = ClawRotation.Center;
        } else if (b){
            robotState.ClawRotation = ClawRotation.RightPickup;
        }
    }

    public void setClawProperties(boolean bumper_left, boolean bumper_right, float right_trigger, float left_trigger){
        if (bumper_left){
            if (ClawRotation.RightPickup == robotState.ClawRotation){
                robotState.ClawRotation = ClawRotation.RightDropoff;
            }
            else if (ClawRotation.Center == robotState.ClawRotation){
                robotState.ClawRotation = ClawRotation.RightPickup;
            }
            else if (ClawRotation.LeftDropoff == robotState.ClawRotation){
                robotState.ClawRotation = ClawRotation.Center;
            }
            else if (ClawRotation.LeftPickup == robotState.ClawRotation){
                robotState.ClawRotation = ClawRotation.Center;
            }
        }
        else if (bumper_right){
            if (ClawRotation.Center == robotState.ClawRotation){
                robotState.ClawRotation = ClawRotation.LeftPickup;
            }
            else if (ClawRotation.LeftPickup == robotState.ClawRotation){
                robotState.ClawRotation = ClawRotation.LeftDropoff;
            }
            else if (ClawRotation.RightDropoff == robotState.ClawRotation){
                robotState.ClawRotation = ClawRotation.Center;
            }
            else if (ClawRotation.RightPickup == robotState.ClawRotation){
                robotState.ClawRotation = ClawRotation.Center;
            }
        }
        else if (right_trigger == 1){
            robotState.ClawRotation = ClawRotation.Center;
        }
        else if (left_trigger == 1){
            robotState.ClawRotation = ClawRotation.Center;
        }
    }


    @Override
    public void update() {
        switch (robotState.ClawRotation){
            case Center:
                clawTiltServo.setPosition(robotState.CLAWCENTERTILT);
                break;
            case RightPickup:
                clawTiltServo.setPosition(robotState.CLAWRIGHTPICKUPTILTPOSITION);
                break;
            case LeftPickup:
                        clawTiltServo.setPosition(robotState.CLAWLEFTPICKUPTILTPOSITION);
                break;
            case RightDropoff:
                clawTiltServo.setPosition(robotState.CLAWRIGHTDROPOFFTILTPOSITION);
                break;
            case LeftDropoff:
                clawTiltServo.setPosition(robotState.CLAWLEFTDROPOFFTILTPOSITION);
                break;
        }

    }
}
