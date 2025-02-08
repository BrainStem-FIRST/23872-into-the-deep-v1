package org.firstinspires.ftc.teamcode.auto.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.PIDController;
@Config
public class Extension implements Component {

    // initialization
    private Telemetry telemetry;
    public static DcMotorEx extensionMotor;
    private HardwareMap map;

    private AnalogInput eTrackerEncoder;

    public static class Params {
        // PIDS Values
        public double kP_Up = 0.02;//FIXME
        public double kI_Up = 0.00; //FIXME
        public double kD_Up = 0.000;//FIXME
        public double kS = 0;

        public int TOLERANCE = 40;

        public int EXTENSION_MAX = 600;
        public int EXTENSION_LEFT_BLOCK = 415;
        public int EXTENSION_CENTER_BLOCK = 365;
        public int EXTENSION_RIGHT_BLOCK = 375;
        public int EXTENSION_MIN = 0;
        public int EXTENSION_CUSTOM = 10;
        public static int RETRACT_POSITION = 0;
    }

    // some variables needed for class
    public static int target = 0;


    private double power = 0;

    private int error = 0;
    // instantiating PIDController
    PIDController extensionController;
    DigitalChannel extensionLimitSwitch;

    // Constants

    public static Params PARAMS = new Params();


    // constructor for Extension class
    public Extension(HardwareMap hwMap, Telemetry telemetry) {
        extensionController = new PIDController(PARAMS.kP_Up, PARAMS.kI_Up, PARAMS.kD_Up);
        this.telemetry = telemetry;
        this.map = hwMap;

        extensionController.setInputBounds(PARAMS.EXTENSION_MIN, PARAMS.EXTENSION_MAX);
        extensionController.setOutputBounds(-1.0, 1.0);
        extensionMotor = new CachingMotor(map.get(DcMotorEx.class, "extension"));
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionLimitSwitch = hwMap.get(DigitalChannel.class, "eLimitSwitch");
    }

    // creating enums for Extension
    public enum ExtensionState {
        OUT,
        IN,
        OFF,
        CUSTOM,
        RETRACT
    }

    // creating extensionState var
    public static ExtensionState extensionState = ExtensionState.IN;

    // setting the extension power off
    public void extensionOff() {
        extensionMotor.setPower(0);
    }

    // setting the pid extension power customizable
    private void setExtensionPower(int ticks) {
        target = ticks;
        error = extensionMotor.getCurrentPosition() - ticks;
        if (!(Math.abs(error) <= PARAMS.TOLERANCE)) {
            power = extensionController.updateWithError(error) + PARAMS.kS;
        } else {
            power = 0;
        }
        extensionMotor.setPower(-power);
    }

    public void setRetract() {
        extensionState = ExtensionState.RETRACT;
    }

    public void setCustom() {
        extensionState = ExtensionState.CUSTOM;
    }


    public void incrementOut() {
        target += PARAMS.EXTENSION_CUSTOM;
        target = Math.min(target, PARAMS.EXTENSION_MAX);
    }

    public void incrementIn() {
        target -= PARAMS.EXTENSION_CUSTOM;
        target = Math.max(target, PARAMS.EXTENSION_MIN);
    }


    public boolean inTolerance() {
        return Math.abs(extensionMotor.getCurrentPosition() - extensionController.getTarget()) < PARAMS.TOLERANCE;
    }

    // placeholder function
    @Override
    public void reset() {

    }

    public boolean isExtensionLimit() {
        return !extensionLimitSwitch.getState();
    }

    private void setTarget(int target) {
        extensionMotor.setTargetPosition(target);
    }

    public void setMotorPIDPower(int extTicks) {
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = extTicks;
        extensionController.setTarget(extTicks);
        error = extTicks - extensionMotor.getCurrentPosition();
        power = extensionController.updateWithError(error) + PARAMS.kS;
        extensionMotor.setPower(power);;
    }

    private void selectState() {
//        telemetry.addData("Ext State", extensionState);
//        telemetry.addData("Ext Target", target);
//        telemetry.addData("Ext Pos", extensionMotor.getCurrentPosition());
//        telemetry.update();

        switch (extensionState) {
            case RETRACT:
                if (isExtensionLimit()) {
                    extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    target = 0;
                    setCustom();
                } else {
                    extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setMotorPower(-1.0);
                }
                break;

            case OFF:
                setExtensionPower(0);
                break;

            case CUSTOM:
                setMotorPIDPower(target);
                break;

            case OUT:
                target = PARAMS.EXTENSION_MAX;
                if (extensionMotor.getCurrentPosition() < target - 50) {
                    extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setMotorPower(1.0);
                } else {
                    setCustom();
                }
                break;
        }
    }


    private double getControlPower() {
        double pidPower = -extensionController.update(extensionMotor.getCurrentPosition());

        return pidPower;
    }

    public static void setMotorPower(double power) {
        extensionMotor.setPower(power);
    }

    // update function for setting the state to extension
    @Override
    public void update() {
        selectState();
    }

    public String test() {
        return null;
    }

    public class GotoMax implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extensionState = ExtensionState.OUT;
                initialized = true;
            }

            update();
            return extensionMotor.getCurrentPosition() < PARAMS.EXTENSION_MAX - 25;
        }
    }

    public Action gotoMax() {
        return new GotoMax();
    }

    public class GotoRetract implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extensionState = ExtensionState.RETRACT;
                initialized = true;
            }
            update();

            return !inTolerance();
        }
    }

    public Action gotoRetract() {
        return new GotoRetract();
    }

    public Action gotoCenterBlock() {
        return new GotoCenterBlock();
    }

    public class GotoCenterBlock implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                target = PARAMS.EXTENSION_CENTER_BLOCK;
                extensionState = ExtensionState.CUSTOM;
                initialized = true;
            }

            telemetry.addData("Ext Target", target);
            telemetry.addData("Ext Pos", extensionMotor.getCurrentPosition());
            telemetry.update();

            update();

            return !inTolerance();
        }
    }

    public Action gotCenterBlock() {
        return new GotoCenterBlock();
    }

    public Action goToPosition(int targetPosition, int tolerance) {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setExtensionPower(targetPosition);
                return Math.abs(extensionMotor.getCurrentPosition() - target) > tolerance;
            }
        };
    }
}
