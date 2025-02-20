package org.firstinspires.ftc.teamcode.auto.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.CachingMotor;

@Config
public class CollectorAuto implements ComponentAuto {

    public static class Params {
        public double maxAutoCollectTime = 3.0  ;
    }

    Telemetry telemetry;
    HardwareMap hardwareMap;
    CachingMotor collectorMotor;
    public static Params PARAMS = new Params();

    public CollectorState collectorState;
    NormalizedColorSensor colorSensor;
    private int currentCounter = 0;
    private final ElapsedTime extakeExtraTimer = new ElapsedTime();
    public CollectorAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        collectorState = CollectorState.LEVEL;
        collectorMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class, "collector"));
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "collectorColor");
    }

    public enum CollectorState {
        INTAKE,
        EJECT,
        LEVEL

    }

    public double getDistance() {
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
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

    private void collectorOut() {
        collectorMotor.setPower(0.6);
    }

    private void collectorIn() {
        // Define thresholds and constants (if not already defined globally)
        final double CURRENT_THRESHOLD = 10000; // Current threshold in milliamps
        final int JAM_FRAME_COUNT = 10; // Number of consecutive frames to detect a jam
        final double COLLECT_POWER = -0.65; // Power for normal collection
        final double UNJAM_POWER = 0.99; // Power for unjamming (reverse direction)
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

    public void setIntake() {
        collectorState = CollectorState.INTAKE;
    }

    public void setEject() {
        collectorState = CollectorState.EJECT;
    }

    public void setLevel() {
        collectorState = CollectorState.LEVEL;
    }

    public class CollectorIn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                collectorState = CollectorState.INTAKE;
                initialized = true;
            }

            update();

            return false;
        }
    }

    public Action collectorInAction() {
        return new CollectorIn();
    }
    public class CollectorOff implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                collectorState = CollectorState.LEVEL;
                initialized = false;
            }

            update();

            return false;
        }
    }

    public Action collectorOffAction() {
        return new CollectorOff();
    }

    public Action waitForCollectionAction() {
        return new WaitForCollectionAction();
    }

    public class WaitForCollectionAction implements Action {
        ElapsedTime timer = new ElapsedTime();
        boolean first = true;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (first) {
                timer.reset();
                first = false;
            }

            // Get the current distance from the color sensor
            double distance = getDistance();

            // Add telemetry for debugging
            telemetry.addData("Color Sensor Distance", distance);
            telemetry.update();

            packet.put("is finished", distance > 3 && timer.seconds() < PARAMS.maxAutoCollectTime);

            // Return true if the block is detected (distance <= 3)
            return distance > 3 && timer.seconds() < PARAMS.maxAutoCollectTime;
        }
    }
}
