package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.subsystem.Lift;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositGripSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositReleaseSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositSpecimenBlockCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.DepositSpecimenHighBarSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.GrabSpecimenSequenceCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.ResetLiftCommand;
import org.firstinspires.ftc.teamcode.teleop.commandGroups.SpecimenPreDeposit;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftHighBasketCommand;
import org.firstinspires.ftc.teamcode.teleop.commands.liftCommands.LiftLowBasketCommand;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp(name="Lift Testing TeleOp", group="Hippos")
public class LiftTestingTeleOp extends LinearOpMode {
    public Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new Lift(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                lift.liftMotor.setPower(1.0);
            } else if (gamepad1.b) {
                lift.liftMotor.setPower(-1.0);
            } else {
                lift.liftMotor.setPower(0);
            }

            telemetry.addData("Lift Power", lift.liftMotor.getPower());
            telemetry.update();
        }
    }
}
