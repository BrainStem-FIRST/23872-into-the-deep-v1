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
        public double ColorSensorDistance = 3.0 ;
        public double maxAutoCollectTime = 1.0  ;
        public double CURRENT_THRESHOLD = 7500; // Current threshold in milliamps
        public int JAM_FRAME_COUNT = 1; // Number of consecutive frames to detect a jam
        public double COLLECT_POWER = -0.99; // Power for normal collection
        public double UNJAM_POWER = 0.45; // Power for unjamming (reverse direction)
        public double UNJAM_TIMEOUT = 3.0; // Timeout for resetting current counter (in seconds)
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
        //collectorMotor.setPower(-0.6);
        // Define thresholds and constants (if not already defined globally)
        // Check for a current spike indicating a jam
//        if (collectorMotor.getCurrent(CurrentUnit.MILLIAMPS) > PARAMS.CURRENT_THRESHOLD) {
//            currentCounter += 1; // Increment the jam counter
//            extakeExtraTimer.reset(); // Reset the unjam timer
//        } else {
//            // Reset the current counter if no jam is detected for the timeout period
////            if (extakeExtraTimer.seconds() > PARAMS.UNJAM_TIMEOUT) {
////                currentCounter = 0;
////            }
//            currentCounter = 0;
//        }
        if(collectorMotor.getCurrent(CurrentUnit.MILLIAMPS) >= PARAMS.CURRENT_THRESHOLD){
            collectorMotor.setPower(PARAMS.UNJAM_POWER);
        }
        else {
            collectorMotor.setPower(PARAMS.COLLECT_POWER);
        }

        // If a jam is detected for the required number of frames, unjam the collector
//        if (currentCounter >= PARAMS.JAM_FRAME_COUNT) {
//            collectorMotor.setPower(PARAMS.UNJAM_POWER); // Reverse the motor to unjam
//            telemetry.addData("Collector Status", "Unjamming Block");
//        } else {
//            // Otherwise, continue collecting
//            collectorMotor.setPower(PARAMS.COLLECT_POWER);
//            telemetry.addData("Collector Status", "Collecting");
//        }

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
        ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                collectorState = CollectorState.INTAKE;
                initialized = true;
            }

            update();

            return (timer.seconds() <= 1.5);
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

      //      packet.put("is finished", distance > PARAMS.ColorSensorDistance && timer.seconds() < PARAMS.maxAutoCollectTime);

            // Return true if the block is detected (distance <= 3)
            return distance > PARAMS.ColorSensorDistance && timer.seconds() < PARAMS.maxAutoCollectTime;
        }
    }
}
