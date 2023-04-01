package edu.edina.library.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.sql.Time;

import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.PoleLocation;
import edu.edina.library.util.RobotState;
import edu.edina.library.util.ClawRotation;
import edu.edina.library.util.Stickygamepad;

public class Claw extends edu.edina.library.subsystems.Subsystem {
    private ClawRotation clawRotation;
    private Servo clawRotationServo;
    private RobotState robotState;

    public Claw(HardwareMap map, RobotState robotState) {
        clawRotationServo = map.get(Servo.class, "clawRotationServo");
        this.robotState = robotState;
        clawRotation = ClawRotation.Center;
    }

    public void setClawProperties(boolean dpad_left, boolean dpad_up, boolean dpad_right,boolean x, boolean y, boolean b) {
        if (dpad_left) {
            clawRotation = ClawRotation.LeftDropoff;
        } else if(dpad_up){
            clawRotation = ClawRotation.Center;
        } else if(dpad_right){
            clawRotation = ClawRotation.RightDropoff;
        } else if (x){
            clawRotation = ClawRotation.LeftPickup;
        } else if (y){
            clawRotation = ClawRotation.Center;
        } else if (b){
            clawRotation = ClawRotation.RightPickup;
        }
    }

    public void setClawProperties(boolean bumper_left, boolean bumper_right){
        if (bumper_left){
            if (ClawRotation.RightPickup == clawRotation){
                clawRotation = ClawRotation.RightDropoff;
            }
            else if (ClawRotation.Center == clawRotation){
                clawRotation = ClawRotation.RightPickup;
            }
            else if (ClawRotation.LeftDropoff == clawRotation){
                clawRotation = ClawRotation.Center;
            }
            else if (ClawRotation.LeftPickup == clawRotation){
                clawRotation = ClawRotation.Center;
            }
        }
        else if (bumper_right){
            if (ClawRotation.Center == clawRotation){
                clawRotation = ClawRotation.LeftPickup;
            }
            else if (ClawRotation.LeftPickup == clawRotation){
                clawRotation = ClawRotation.LeftDropoff;
            }
            else if (ClawRotation.RightDropoff == clawRotation){
                clawRotation = ClawRotation.Center;
            }
            else if (ClawRotation.RightPickup == clawRotation){
                clawRotation = ClawRotation.Center;
            }
        }
    }


    @Override
    public void update() {
        switch (clawRotation){
            case Center:
                clawRotationServo.setPosition(robotState.CLAWCENTERTILT);
                break;
            case RightPickup:
                clawRotationServo.setPosition(robotState.CLAWRIGHTPICKUPTILTPOSITION);
                break;
            case LeftPickup:
                clawRotationServo.setPosition(robotState.CLAWLEFTPICKUPTILTPOSITION);
                break;
            case RightDropoff:
                clawRotationServo.setPosition(robotState.CLAWRIGHTDROPOFFTILTPOSITION);
                break;
            case LeftDropoff:
                clawRotationServo.setPosition(robotState.CLAWLEFTDROPOFFTILTPOSITION);
                break;
        }

    }
}
