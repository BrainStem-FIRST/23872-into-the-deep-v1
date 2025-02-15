package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.CachingMotor;

@Config
public class CollectorTele implements ComponentTele {
    public static double currentThreshold = 7500, extakeExtraTime = 0.5, outtakePower = -0.40;

    Telemetry telemetry;
    HardwareMap hardwareMap;
    CachingMotor collectorMotor;
    private int currentCounter = 0;
    private final ElapsedTime extakeExtraTimer = new ElapsedTime();

    public CollectorState collectorState;

    public CollectorTele(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        collectorState = CollectorState.LEVEL;
        collectorMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class, "collector"));
        collectorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public enum CollectorState {
        INTAKE,
        EJECT,
        LEVEL

    }

    @Override
    public void reset() {}

    @Override
    public String test() {
        return "true";
    }

    @Override
    public void update() {
        switch (collectorState) {
            case LEVEL: {
                collectorLevel();
                break;
            }

            case INTAKE: {
                collectorIn();
                break;
            }

            case EJECT: {
                collectorOut();
                break;
            }
        }
    }

    public double getPower(){
        return collectorMotor.getPower();
    }
    public CollectorState getState(){
        return collectorState;
    }
    private void collectorLevel() {
        collectorMotor.setPower(0.0);
    }
    private void collectorIn() {
        // Define thresholds and constants (if not already defined globally)
        final double CURRENT_THRESHOLD = 7500; // Current threshold in milliamps
        final int JAM_FRAME_COUNT = 10; // Number of consecutive frames to detect a jam
        final double COLLECT_POWER = 0.75; // Power for normal collection
        final double UNJAM_POWER = -0.99; // Power for unjamming (reverse direction)
        final double UNJAM_TIMEOUT = 0.5; // Timeout for resetting current counter (in seconds)

        // Check for a current spike indicating a jam
        if (collectorMotor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD) {
            currentCounter += 1; // Increment the jam counter
            extakeExtraTimer.reset(); // Reset the unjam timer
        } else {
            // Reset the current counter if no jam is detected for the timeout period
            if (extakeExtraTimer.seconds() > UNJAM_TIMEOUT) {
                currentCounter = 0;
            }
        }

        // If a jam is detected for the required number of frames, unjam the collector
        if (currentCounter > JAM_FRAME_COUNT) {
            collectorMotor.setPower(UNJAM_POWER); // Reverse the motor to unjam
            telemetry.addData("Collector Status", "Unjamming Block");
        } else {
            // Otherwise, continue collecting
            collectorMotor.setPower(COLLECT_POWER);
            telemetry.addData("Collector Status", "Collecting");
        }

        // Add telemetry for debugging
        telemetry.addData("Collector Current (mA)", collectorMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Current Counter", currentCounter);
        telemetry.addData("Unjam Timer (s)", extakeExtraTimer.seconds());
        telemetry.update();
    }

    private void collectorOut() {
        collectorMotor.setPower(-0.99);
    }

    public void setIntake() {
        collectorState = CollectorState.INTAKE;
    }

    public void setEject() {
        collectorState = CollectorState.EJECT;
    }

    public void setLevel() {
        collectorState = CollectorState.LEVEL;
    }

}
