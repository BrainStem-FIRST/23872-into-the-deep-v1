package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDPatternCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.auto.subsystem.ExtensionAuto;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;


@Config
@Autonomous(name="Yellow Blocks", group="Hippos")
public class YellowBlocks extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(-39, -64, Math.toRadians(0));
        Pose2d depositPose = new Pose2d(-60, -59, Math.toRadians(65));
        Pose2d rightBlockPose = new Pose2d(-46.5, -44, Math.toRadians(90));
        Pose2d centerBlockPose = new Pose2d(-56.5, -43.5, Math.toRadians(89));
        Pose2d leftBlockPose = new Pose2d(-50.5, -32, Math.toRadians(155));
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
        robot.extension.reset();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                robot.lift.gotoHighBasket(),
                                new SleepAction(1.5),
                                robot.depositor.gotoUp(),
                                depositPreloadApproach
                        ),
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.2),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.15),

                        // RIGHT BLOCK

                        // COLLECT SEQUENCE
                        new ParallelAction(
                                rightBlock,
                                robot.extension.goToPosition(ExtensionAuto.PARAMS.EXTENSION_RIGHT_BLOCK, ExtensionAuto.PARAMS.TOLERANCE),
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        robot.collector.collectorInAction(),
                                        new SleepAction(2.0),
                                        new ParallelAction(
                                                robot.lift.gotoDeconflict(),
                                                robot.depositor.gotoDown()
                                        )
                                )
                        ),
                        robot.extension.goToPosition(0, ExtensionAuto.PARAMS.TOLERANCE),
                        robot.collector.collectorOffAction(),

//                         DEPOSIT SEQUENCE
                        new SleepAction(0.5),
                        robot.lift.goToGrabFast(),
                        new SleepAction(0.1),
                        robot.depositor.closeClaw(),
                        new SleepAction(0.25),
                        robot.lift.gotoDeconflict(),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.2),
                        robot.lift.gotoHighBasket(),
                        depositRightBlock,
                        new SleepAction(0.1),

                        // RETRACT SEQUENCE
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.1),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),

                        // CENTER BLOCK


                        new ParallelAction(
                                centerBlock,
                                robot.extension.goToPosition(ExtensionAuto.PARAMS.EXTENSION_CENTER_BLOCK, ExtensionAuto.PARAMS.TOLERANCE),
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        robot.collector.collectorInAction(),
                                        new SleepAction(2.0),
                                        new ParallelAction(
                                                robot.lift.gotoDeconflict(),
                                                robot.depositor.gotoDown()
                                        )
                                )
                        ),
                        robot.extension.goToPosition(0, ExtensionAuto.PARAMS.TOLERANCE),
                        robot.collector.collectorOffAction(),

//                         DEPOSIT SEQUENCE
                        new SleepAction(0.5),
                        robot.lift.goToGrabFast(),
                        new SleepAction(0.1),
                        robot.depositor.closeClaw(),
                        new SleepAction(0.25),
                        robot.lift.gotoDeconflict(),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.2),
                        robot.lift.gotoHighBasket(),
                        depositCenterBlock,
                        new SleepAction(0.1),

                        // RETRACT SEQUENCE
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.1),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),

                        // LEFT BLOCK

                        // COLLECT SEQUENCE
                        new ParallelAction(
                                leftBlock,
                                robot.extension.goToPosition(ExtensionAuto.PARAMS.EXTENSION_LEFT_BLOCK, ExtensionAuto.PARAMS.TOLERANCE),
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        robot.collector.collectorInAction(),
                                        new SleepAction(2.5),
                                        new ParallelAction(
                                                robot.lift.gotoDeconflict(),
                                                robot.depositor.gotoDown()
                                        )
                                )
                        ),
                        robot.extension.goToPosition(0, ExtensionAuto.PARAMS.TOLERANCE),
                        robot.collector.collectorOffAction(),


                        // DEPOSIT SEQUENCE
                        new SleepAction(0.5),
                        robot.lift.goToGrabFast(),
                        new SleepAction(0.1),
                        robot.depositor.closeClaw(),
                        new SleepAction(0.25),
                        robot.lift.gotoDeconflict(),
                        new SleepAction(0.1),
                        robot.depositor.gotoUp(),
                        new SleepAction(0.15),
                        robot.lift.gotoHighBasket(),
                        depositLeftBlock,
                        new SleepAction(0.1),

                        // RETRACT SEQUENCE
                        robot.depositor.gotoBackward(),
                        new SleepAction(0.2),
                        robot.depositor.openClaw(),
                        new SleepAction(0.25),
                        robot.depositor.gotoUp(),
                        park,

                        // PARK
                        new ParallelAction(
                                robot.depositor.openClaw(),
                                robot.depositor.gotoDown()
                        )

//                        robot.extension.gotoLeftBlock()

                )

        );
    }
}
