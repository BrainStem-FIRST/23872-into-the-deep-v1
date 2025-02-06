package org.firstinspires.ftc.teamcode.teleop.subsystem;

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
import org.firstinspires.ftc.teamcode.auto.subsystem.Lift;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.PIDController;
@Config
public class Extension implements Component {

    // initialization
    private Telemetry telemetry;
    public DcMotorEx extension;
    private HardwareMap map;

    private AnalogInput eTrackerEncoder;

    public static class Params {
        // PIDS Values
        public double kP_Up = 0.05;//FIXME
        public double kI_Up = 0.00; //FIXME
        public double kD_Up = 0.000;//FIXME
        public double kS = 0;

        public double FIND_BLOCK_POWER = 0.75;

        public int TOLERANCE = 5;

        public static final int EXTENSION_MAX = 500;
        public static final int EXTENSION_MIN = 0;
        public int EXTENSION_CUSTOM = 10;
        public static final int RETRACT_POSITION = 0;
    }

    // some variables needed for class
    public int target = 0;


    private double power = 0;

    private int error = 0;
    public double targetFindBlockPower = 0;
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
        extension = new CachingMotor(map.get(DcMotorEx.class, "extension"));
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public ExtensionState extensionState = ExtensionState.IN;

    // setting the extension power off
    public void extensionOff(){
        extension.setPower(0);
    }
    // setting the pid extension power customizable
    private void setExtensionPower(int ticks){
        target = ticks;
        error = extension.getCurrentPosition() - ticks;
        if(!(Math.abs(error) <= PARAMS.TOLERANCE)) {
            power = extensionController.updateWithError(error) + PARAMS.kS;
        }
        else {
            power = 0;
        }
        extension.setPower(-power);
    }

    public void setRetract(){
        telemetry.addData("Extension State set", "RETRACT");
        extensionState = ExtensionState.RETRACT;
    }

    public void setCustom(){ extensionState = ExtensionState.CUSTOM;}


    public void incrementOut(){
        target += PARAMS.EXTENSION_CUSTOM;
        target = Math.min(target, PARAMS.EXTENSION_MAX);
    }

    public void incrementIn(){
        target -= PARAMS.EXTENSION_CUSTOM;
        target = Math.max(target, PARAMS.EXTENSION_MIN);
    }


    public boolean inTolerance() {
        return Math.abs(extension.getCurrentPosition() - extensionController.getTarget()) < PARAMS.TOLERANCE;
    }

    // placeholder function
    @Override
    public void reset() {
        target = 0;
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isExtensionLimit() {
        return !extensionLimitSwitch.getState();
    }

    private void setTarget(double target) {
        extensionController.setTarget(target);
    }

    private void selectState() {
        switch (extensionState) {
            case RETRACT:
//                if (isExtensionLimit()) {
//                    extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    target = 0;
//                    setCustom();
//                } else {

//                if (extension.getCurrentPosition() > 35) {
                    setMotorPower(-1.0);
//                } else {
//                    setMotorPower(-0.2);
//                }
//                }
                break;

            case OFF:
                setExtensionPower(0);
                break;

            case CUSTOM:
                //setTarget(target);
                //setMotorPower(getControlPower());
                extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(extension.getCurrentPosition() > Params.EXTENSION_MAX)
                     setMotorPower(-0.2);
                else
                    extension.setPower(targetFindBlockPower);
                telemetry.addData("extension power", extension.getPower());
                break;
        }
    }


    public double getControlPower() {

        double pidPower = -extensionController.update(extension.getCurrentPosition());

        return pidPower;
    }

    public void setMotorPower(double power) {
        extension.setPower(power);
    }

    // update function for setting the state to extension
    @Override
    public void update() {
        selectState();
        telemetry.addData("Extension Position", extension.getCurrentPosition());
        telemetry.addData("Extension Power", extension.getPower());
        telemetry.addData("Extension Limit", isExtensionLimit());
        telemetry.addData("Extension State", extensionState);
        telemetry.addData("Extension Target", target);
        telemetry.addData("Ext Limit", isExtensionLimit());
    }

    public String test() {
        return null;
    }

}
