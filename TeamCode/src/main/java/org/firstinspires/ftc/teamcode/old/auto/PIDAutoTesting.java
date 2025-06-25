package org.firstinspires.ftc.teamcode.old.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.old.drivetrain.PIDDrivetrain;

@Disabled
@Autonomous(name="PID Auto Testing", group="Hippos")
public class PIDAutoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -66, -30, AngleUnit.DEGREES, 0);
        Pose2D approachPose1 = new Pose2D(DistanceUnit.INCH, 0, -45, AngleUnit.DEGREES, 0);
        Pose2D targetPose1 = new Pose2D(DistanceUnit.INCH, -62, -48, AngleUnit.DEGREES, 0);
        Pose2D approachPose2 = new Pose2D(DistanceUnit.INCH, 0, -52, AngleUnit.DEGREES, 0);
        Pose2D targetPose2 = new Pose2D(DistanceUnit.INCH, -62, -52, AngleUnit.DEGREES, 0);


        PIDDrivetrain drive = new PIDDrivetrain(hardwareMap, telemetry, initialPose);
        drive.resetPosition(initialPose);

        drive.addPathPoint(approachPose1, 2.0, 2, 0, 5);
        drive.addPathPoint(targetPose1, 2.0, 2, 0, 5);
        drive.addPathPoint(targetPose1, 2.0, 2, 0, 5);
        drive.addPathPoint(approachPose2, 2.0, 2, 0,5);
        drive.addPathPoint(targetPose2, 1.0, 2);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.drivePath();
        }
    }
}
