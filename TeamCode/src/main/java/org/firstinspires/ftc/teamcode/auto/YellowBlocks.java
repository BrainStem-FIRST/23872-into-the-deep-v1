package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;

@Autonomous(name="Yellow Blocks", group="Hippos")
public class YellowBlocks extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(-39, -64, Math.toRadians(0));
        Pose2d depositPose = new Pose2d(-59, -59, Math.toRadians(65));
        Pose2d rightBlockPose = new Pose2d(-46.5, -44, Math.toRadians(90));
        Pose2d centerBlockPose = new Pose2d(-58, -43.5, Math.toRadians(92));
        Pose2d leftBlockPose = new Pose2d(-52, -32, Math.toRadians(160));
        Pose2d parkPose = new Pose2d(-20, -12, Math.toRadians(0));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, beginPose);
        PinpointDrive drive = robot.drive;

        TrajectoryActionBuilder depositPreloadTrajectory = drive.actionBuilder(beginPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, Math.toRadians(235));

        TrajectoryActionBuilder rightBlockTrajectory = drive.actionBuilder(depositPose)
                .afterDisp(4, robot.lift.gotoDeconflict())
                .splineToLinearHeading(rightBlockPose, Math.toRadians(90));

        TrajectoryActionBuilder centerBlockTrajectory = drive.actionBuilder(depositPose)
                .afterDisp(4, robot.lift.gotoDeconflict())
                .splineToLinearHeading(centerBlockPose, Math.toRadians(90));

        TrajectoryActionBuilder leftBlockTrajectory = drive.actionBuilder(depositPose)
                .afterDisp(4, robot.lift.gotoDeconflict())
                .splineToLinearHeading(leftBlockPose, Math.toRadians(110));

        TrajectoryActionBuilder depositRightBlockTrajectory = drive.actionBuilder(rightBlockPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, Math.toRadians(245));

        TrajectoryActionBuilder depositCenterBlockTrajectory = drive.actionBuilder(centerBlockPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, Math.toRadians(245));

        TrajectoryActionBuilder depositLeftBlockTrajectory = drive.actionBuilder(leftBlockPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, Math.toRadians(245));

        TrajectoryActionBuilder parkTrajectory = drive.actionBuilder(depositPose)
                .afterDisp(4, robot.lift.gotoDeconflict())
                .splineToLinearHeading(parkPose, Math.toRadians(0));

        Action depositPreloadApproach = depositPreloadTrajectory.build();
        Action rightBlock = rightBlockTrajectory.build();
        Action centerBlock = centerBlockTrajectory.build();
        Action leftBlock = leftBlockTrajectory.build();
        Action depositRightBlock = depositRightBlockTrajectory.build();
        Action depositCenterBlock = depositCenterBlockTrajectory.build();
        Action depositLeftBlock = depositLeftBlockTrajectory.build();
        Action park = parkTrajectory.build();

        Actions.runBlocking(
                robot.depositor.closeClaw()
        );

        telemetry.addLine("Robot Ready");
        telemetry.update();

        robot.lift.reset();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                robot.extension.gotoRetract(),
                                robot.lift.gotoHighBasket(),
                                new SleepAction(2.0),
                                robot.depositor.gotoUp(),
                                depositPreloadApproach
                        ),
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.35),
                        robot.depositor.openClaw(),
                        new SleepAction(0.5),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.25),

                        // RIGHT BLOCK
                        rightBlock,

                        // COLLECT SEQUENCE
                        new ParallelAction(
                                robot.lift.gotoDeconflict(),
                                robot.depositor.gotoDown(),
                                robot.collector.collectorInAction(),
                                robot.extension.gotoCenterBlock()
                        ),
                        new SleepAction(1.5),
                        robot.extension.gotoRetract(),
                        robot.collector.collectorOffAction(),

                        // DEPOSIT SEQUENCE
                        robot.lift.gotoGrab(),
                        robot.depositor.closeClaw(),
                        new SleepAction(0.25),
                        robot.lift.gotoDeconflict(),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.25),
                        robot.lift.gotoHighBasket(),
                        depositRightBlock,
                        new SleepAction(0.25),

                        // RETRACT SEQUENCE
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.25),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.25),

                        // CENTER BLOCK
                        centerBlock,

                        // COLLECT SEQUENCE
                        new ParallelAction(
                                robot.lift.gotoDeconflict(),
                                robot.depositor.gotoDown(),
                                robot.collector.collectorInAction(),
                                robot.extension.gotoLeftBlock()
                        ),
                        new SleepAction(1.5),
                        robot.extension.gotoRetract(),
                        robot.collector.collectorOffAction(),

                        // DEPOSIT SEQUENCE
                        robot.lift.gotoGrab(),
                        robot.depositor.closeClaw(),
                        new SleepAction(0.35),
                        robot.lift.gotoDeconflict(),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.25),
                        robot.lift.gotoHighBasket(),
                        new SleepAction(0.25),
                        depositCenterBlock,

                        // RETRACT SEQUENCE
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.5),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.25),

                        // LEFT BLOCK
                        leftBlock,

                        // COLLECT SEQUENCE
                        new ParallelAction(
                                robot.lift.gotoDeconflict(),
                                robot.depositor.gotoDown(),
                                robot.collector.collectorInAction(),
                                robot.extension.gotoLeftBlock()
                        ),
                        new SleepAction(1.5),
                        robot.extension.gotoRetract(),
                        robot.collector.collectorOffAction(),

                        // DEPOSIT SEQUENCE
                        robot.lift.gotoGrab(),
                        robot.depositor.closeClaw(),
                        new SleepAction(0.35),
                        robot.lift.gotoDeconflict(),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.25),
                        robot.lift.gotoHighBasket(),
                        new SleepAction(0.25),
                        depositLeftBlock,

                        // RETRACT SEQUENCE
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.25),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.25),

                        // PARK
                        new ParallelAction(
                                park,
                                robot.depositor.gotoDown(),
                                robot.depositor.openClaw()
                        )
//                        robot.extension.gotoLeftBlock()

                )
        );

    }
}