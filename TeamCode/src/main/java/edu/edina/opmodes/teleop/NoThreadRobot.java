package edu.edina.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.edina.library.subsystems.Claw;

import java.util.ArrayList;
import java.util.List;

import edu.edina.library.subsystems.Claw;
import edu.edina.library.subsystems.Lift;
import edu.edina.library.subsystems.MecanumDrive;
import edu.edina.library.subsystems.Subsystem;
import edu.edina.library.util.ClawRotation;
import edu.edina.library.util.RobotHardware;
import edu.edina.library.util.RobotState;

public class NoThreadRobot {
    public Claw claw;
    private List<Subsystem> subsystems;
    private Telemetry telemetry;
    public MecanumDrive driveRR;
    public Lift lift;
    public RobotState robotState = new RobotState();
    public RobotHardware robotHardware = null;

    public void update() {
        for (Subsystem subsystem : subsystems) {
            if (subsystem == null) {
                continue;
            }

            subsystem.update();
        }
    }

    public NoThreadRobot(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        robotHardware = new RobotHardware(map);

        subsystems = new ArrayList<>();

        driveRR = new MecanumDrive(map, robotState);
        subsystems.add(driveRR);

        lift = new Lift(robotState, robotHardware);
        subsystems.add(lift);

        claw = new Claw(robotState, robotHardware);
        subsystems.add(claw);
    }


    public void telemetry()
    {
        robotState.telemetry(telemetry);
        telemetry.update();
    }
}
