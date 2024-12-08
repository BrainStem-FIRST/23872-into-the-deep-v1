package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.subsystem.Depositor;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;

@Autonomous(name="Blue - Blue Clips", group="Hippos")
public class BlueSideBlueClips extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(7.5, -65.6, Math.toRadians(90));
        Pose2d depositPose = new Pose2d(0, -30.5, Math.toRadians(90));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, beginPose);
        PinpointDrive drive = robot.drive;

        TrajectoryActionBuilder depositPreloadTrajectory = drive.actionBuilder(beginPose)
                .splineToLinearHeading(depositPose, Math.toRadians(210));

        Action depositPreloadApproach = depositPreloadTrajectory.build();
//        Actions.runBlocking(
//                robot.depositor.closeClaw(),
//                robot.depositor.gotoBackward(),
//                robot.lift.setSpecimenLevel(),

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction(
                                depositPreloadApproach
                        )
                )
        );
    }

}
