package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystem.CollectorTele;
import org.firstinspires.ftc.teamcode.teleop.subsystem.ComponentTele;
import org.firstinspires.ftc.teamcode.teleop.subsystem.DepositorTele;
import org.firstinspires.ftc.teamcode.teleop.subsystem.ExtensionTele;
import org.firstinspires.ftc.teamcode.teleop.subsystem.LiftTele;

import java.util.ArrayList;

public class BrainSTEMRobot {
    Telemetry telemetry;
    HardwareMap map;
    ArrayList<ComponentTele> subsystems;
    public LiftTele lift;
    public DepositorTele depositor;
    public CollectorTele collector;
    public ExtensionTele extension;

    public PinpointDrive drive;

    public BrainSTEMRobot(Telemetry telemetry, HardwareMap map){
        this.telemetry = telemetry;
        this.map = map;

        subsystems = new ArrayList<>();
        lift = new LiftTele(map, telemetry);
        depositor = new DepositorTele(map, telemetry);
        collector = new CollectorTele(map, telemetry);
        extension = new ExtensionTele(map, telemetry);
        drive = new PinpointDrive(map, new Pose2d(0,0,0));

        subsystems.add(lift);
        subsystems.add(depositor);
        subsystems.add(collector);
        subsystems.add(extension);
    }

    public void update() {
        for (ComponentTele c : subsystems) {
            c.update();
        }
        drive.updatePoseEstimate();
        CommandScheduler.getInstance().run();
    }
}
