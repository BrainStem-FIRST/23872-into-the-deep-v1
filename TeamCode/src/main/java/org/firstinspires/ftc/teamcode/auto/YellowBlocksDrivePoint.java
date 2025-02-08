package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.subsystem.Extension;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;


@Config
@Autonomous(name="Yellow Blocks Drive Point", group="Hippos")
public class YellowBlocksDrivePoint extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(-39, -64, Math.toRadians(0));
        Pose2d depositPose = new Pose2d(-54, -61, Math.toRadians(45));
        Pose2d humanPose = new Pose2d(26, -64, Math.toRadians(0));
        Pose2d rightBlockPose = new Pose2d(-47, -44, Math.toRadians(90));
        Pose2d centerBlockPose = new Pose2d(-57, -43.5, Math.toRadians(89));
        Pose2d leftBlockPose = new Pose2d(-52, -32, Math.toRadians(155));
        Pose2d parkPose = new Pose2d(-20, -12, Math.toRadians(0));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, beginPose);
        PinpointDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        robot.lift.reset();
        robot.depositor.closeClaw();
        robot.depositor.gotoUp();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                robot.extension.goToPosition(0, 5),
                                robot.lift.gotoHighBasket(),
                                autoCommands.driveToPoint(depositPose.position.x, depositPose.position.y, depositPose.heading.toDouble(), 0.5, 0.5, Math.toRadians(2), true, 0.0)
                        ),
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.35),
                        robot.depositor.openClaw(),
                        new SleepAction(0.35),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.35),
                        autoCommands.driveToPoint(beginPose.position.x, beginPose.position.y, beginPose.heading.toDouble(), 1.5, 1.5, 1,false,0.0),
                        autoCommands.driveToPoint(humanPose.position.x, humanPose.position.y, humanPose.heading.toDouble(), 1.0, 1.0, 1,true,0.0)
                )
        );

    }
}