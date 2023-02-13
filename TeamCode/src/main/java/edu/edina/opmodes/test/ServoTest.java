package edu.edina.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "ServoTest")
@Disabled
public class ServoTest extends OpMode {

    private CRServo leftServo;
    private CRServo rightServo;

    @Override
    public void init() {
      leftServo = hardwareMap.crservo.get("leftServo");
      rightServo = hardwareMap.crservo.get("rightServo");
    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper) {
            leftServo.setPower(1);
            rightServo.setPower(-1);
        } else if(gamepad1.right_bumper) {
            leftServo.setPower(-1);
            rightServo.setPower(1);
        } else {
            leftServo.setPower(0);
            rightServo.setPower(0);
        }
    }
}
