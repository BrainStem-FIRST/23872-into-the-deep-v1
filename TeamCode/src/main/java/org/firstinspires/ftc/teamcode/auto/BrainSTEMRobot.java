package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.auto.subsystem.CollectorAuto;
import org.firstinspires.ftc.teamcode.auto.subsystem.ComponentAuto;
import org.firstinspires.ftc.teamcode.auto.subsystem.DepositorAuto;
import org.firstinspires.ftc.teamcode.auto.subsystem.ExtensionAuto;
import org.firstinspires.ftc.teamcode.auto.subsystem.LiftAuto;

import java.util.ArrayList;

@Config
public class BrainSTEMRobot {
    public static int HANG_PARK_ENCODER = 200, HANG_PARK_TOLERANCE = 20;
    Telemetry telemetry;
    HardwareMap map;
    ArrayList<ComponentAuto> subsystems;
    public LiftAuto lift;
    public DepositorAuto depositor;
    public CollectorAuto collector;
    public ExtensionAuto extension;
    public DcMotorEx hangMotor;

    public PinpointDrive drive;



    public BrainSTEMRobot(Telemetry telemetry, HardwareMap map, Pose2d pose){
        this.telemetry = telemetry;
        this.map = map;

        subsystems = new ArrayList<>();
        lift = new LiftAuto(map, telemetry);
        depositor = new DepositorAuto(map, telemetry);
        collector = new CollectorAuto(map, telemetry);
        extension = new ExtensionAuto(map, telemetry);
        drive = new PinpointDrive(map, pose);
        hangMotor = map.get(DcMotorEx.class, "HangMotor");
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setTargetPosition(hangMotor.getCurrentPosition());
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        subsystems.add(lift);
        subsystems.add(depositor);
        subsystems.add(collector);
        subsystems.add(extension);

    }

    public void update() {
        for (ComponentAuto c : subsystems) {
            c.update();
        }
        drive.updatePoseEstimate();
        CommandScheduler.getInstance().run();
    }

    public Action moveHangToPark() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hangMotor.setTargetPosition(HANG_PARK_ENCODER);
                return Math.abs(hangMotor.getCurrentPosition() - HANG_PARK_ENCODER) > HANG_PARK_TOLERANCE;
            }
        };
    }

}
