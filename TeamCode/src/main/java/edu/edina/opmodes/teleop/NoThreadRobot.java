package edu.edina.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.library.subsystems.Claw;

import java.util.ArrayList;
import java.util.List;

import edu.edina.library.subsystems.Claw;
import edu.edina.library.subsystems.Lift;
import edu.edina.library.subsystems.MecanumDrive;
import edu.edina.library.subsystems.Subsystem;
import edu.edina.library.util.ClawRotation;
import edu.edina.library.util.RobotState;

public class NoThreadRobotRR {
    private Claw Claw;
    private List<Subsystem> subsystems;
    private Telemetry telemetry;
    public MecanumDrive driveRR;
    public Lift lift;
    public RobotState robotState = new RobotState();

    public void update() {
        for (Subsystem subsystem : subsystems) {
            if (subsystem == null) continue;
                subsystem.update();
        }
    }

    public NoThreadRobot(OpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;

        subsystems = new ArrayList<>();

        driveRR = new MecanumDrive(opMode.hardwareMap, robotState);
        subsystems.add(driveRR);

        lift = new Lift(opMode.hardwareMap, robotState);
        subsystems.add(lift);

        Claw = new Claw(opMode.hardwareMap, robotState);
        subsystems.add(Claw);
    }


    public void telemetry()
    {
        robotState.telemetry(telemetry);
        telemetry.update();
    }
}
