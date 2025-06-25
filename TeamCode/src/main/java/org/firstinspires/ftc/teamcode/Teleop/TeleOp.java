package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.old.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.old.teleop.commandGroups.DepositGripSequenceCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commandGroups.DepositReleaseSequenceCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commandGroups.DepositSpecimenBlockCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commandGroups.DepositSpecimenHighBarSequenceCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commandGroups.GrabSpecimenSequenceCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commandGroups.ResetLiftCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commandGroups.SpecimenPreDeposit;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.LiftLowBasketCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.depositorCommands.DepositorBackCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.depositorCommands.DepositorHighBasketCommand;
import org.firstinspires.ftc.teamcode.old.teleop.commands.liftCommands.LiftHighBasketCommand;

@Config
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

    }
        private void updateDriver1(BrainSTEMRobot robot) {
            driver1CollectorControls(robot);
            driver1ExtensionControls(robot);
    }

    private void driver1ExtensionControls(BrainSTEMRobot robot) {
        if (gamepad1.dpad_up) {
            robot.extension.incrementOut();
            robot.extension.setCustom();
        } else if (gamepad1.dpad_down) {
            robot.extension.incrementIn();
            robot.extension.setCustom();
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
}