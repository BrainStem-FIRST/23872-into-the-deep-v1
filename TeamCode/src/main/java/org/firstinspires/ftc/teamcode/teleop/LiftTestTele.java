package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="LiftTestTele")
public class LiftTestTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad2.dpad_up)
                liftMotor.setPower(1.0);
            else if (gamepad2.dpad_down)
                liftMotor.setPower(-1.0);
            else liftMotor.setPower(0);
        }
    }

}
