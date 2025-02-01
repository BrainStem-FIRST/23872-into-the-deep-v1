package org.firstinspires.ftc.teamcode.auto.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.CachingMotor;

@Config
public class Collector implements Component {
    public static class Params {

    }

    Telemetry telemetry;
    HardwareMap hardwareMap;
    CachingMotor collectorMotor;
    public static Params PARAMS = new Params();

    public CollectorState collectorState;
    NormalizedColorSensor colorSensor;
    public Collector(HardwareMap hardwareMap, Telemetry telemetry) {
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
        collectorMotor.setPower(1.0);
    }

    private void collectorIn() {
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
                initialized = true;
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

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {


            return getDistance() > 3;
        }
    }
}
