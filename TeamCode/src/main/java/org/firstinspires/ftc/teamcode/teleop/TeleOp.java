package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositGripSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositReleaseSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositSpecimenBlockCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositSpecimenHighBarSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.GrabSpecimenSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.ResetLiftCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.SpecimenPreDeposit;
import org.firstinspires.ftc.teamcode.teleop.commands.depositorCommands.DepositorHighBasketCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftHighBasketCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftLowBasketCommand;
import org.firstinspires.ftc.teamcode.util.Drawing;

public class TeleOp extends LinearOpMode {
    ResetLiftCommand resetLiftCommand;
    ElapsedTime timer;
    boolean liftResetInProgress = false;
    boolean extensionResetInProgress = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, gamepad1);
        resetLiftCommand = new ResetLiftCommand(robot, telemetry);
        timer = new ElapsedTime();

        robot.depositor.setGripperOpen();
        waitForStart();

        while (opModeIsActive()) {
            robot.update();
            updateDrive(robot);
            updateDriver1(robot);
            telemetry.addData("lift state", robot.lift.liftState);
            telemetry.update();
        }
    }


    private void updateDriver1(BrainSTEMRobot robot) {
        driver2LiftControls(robot);
        driver2DepositorControls(robot);
        driver1CollectorControls(robot);
        driver1ExtensionControls(robot);
        driver2DepositorControls(robot);
    }

    private void driver1ExtensionControls(BrainSTEMRobot robot) {
        if (gamepad1.dpad_up) {
            robot.extension.incrementOut();
            robot.extension.setCustom();
        } else if (gamepad1.dpad_down) {
            robot.extension.incrementIn();
            robot.extension.setCustom();
        }

        if (gamepad1.x && !extensionResetInProgress) {
            robot.extension.setRetract();
            extensionResetInProgress = true;
        } else if (extensionResetInProgress && !gamepad1.x) {
            extensionResetInProgress = false;
            robot.extension.reset();
            robot.extension.setCustom();
        }
    }

    private void driver2LiftControls(BrainSTEMRobot robot) {
        if (gamepad2.dpad_up) {
            new LiftHighBasketCommand(robot.lift, telemetry).schedule();
            new DepositorHighBasketCommand(robot.depositor, telemetry).schedule();
        } else if (gamepad2.dpad_down) {
            new LiftLowBasketCommand(robot.lift, telemetry).schedule();
        }
        if (gamepad2.y) {
            new DepositSpecimenHighBarSequenceCommand(robot, telemetry).schedule();
        }

        if (gamepad2.x && !liftResetInProgress) {
            new ResetLiftCommand(robot, telemetry).schedule();
            liftResetInProgress = true;
        } else if (liftResetInProgress && !gamepad2.x) {
            liftResetInProgress = false;
            robot.lift.reset();
            robot.lift.setDeconflict();
        }
    }

    private void driver1CollectorControls(BrainSTEMRobot robot){

        if (gamepad1.b) {
            robot.collector.setEject();
        } else if (gamepad1.a) {
            robot.collector.setIntake();
        }
        else {
            robot.collector.setLevel();
        }
    }

    private void driver2DepositorControls(BrainSTEMRobot robot) {
        if (gamepad2.left_bumper) {
            new DepositReleaseSequenceCommand(robot, telemetry).schedule();
        }

        if (gamepad2.right_bumper) {
            new DepositGripSequenceCommand(robot, telemetry).schedule();
        }
        if (gamepad2.left_trigger > 0.5) {
            new GrabSpecimenSequenceCommand(robot, telemetry).schedule();
        } else if (gamepad2.right_trigger > 0.5) {
            new SpecimenPreDeposit(robot, telemetry).schedule();

        }

        if (gamepad2.a) {
            new DepositSpecimenBlockCommand(robot,telemetry).schedule();
        }
    }

    private void updateDriver2(BrainSTEMRobot robot) {
    }

    private void drawRobot(BrainSTEMRobot robot) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), robot.drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void updateDrive(BrainSTEMRobot robot) {
        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        telemetry.addData("x", robot.drive.pose.position.x);
        telemetry.addData("y", robot.drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.pose.heading.toDouble()));
        telemetry.addData("Extension Pow", robot.extension.extension.getPower());

        drawRobot(robot);
    }
}
