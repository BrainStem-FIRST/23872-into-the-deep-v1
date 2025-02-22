package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.subsystem.ComponentTele;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
@TeleOp(name="ColorSensorTestTele")
public class ColorSensorTestTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "collectorColor");

        waitForStart();

        while (opModeIsActive()) {
            double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            telemetry.addData("distance in trough", "" + dist);
            telemetry.update();
        }
    }
}
