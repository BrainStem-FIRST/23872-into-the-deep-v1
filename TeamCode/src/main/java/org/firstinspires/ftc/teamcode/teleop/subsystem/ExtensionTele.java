package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.PIDController;
@Config
public class ExtensionTele implements ComponentTele {

    // initialization
    private Telemetry telemetry;
    public DcMotorEx extension;
    private HardwareMap map;

    private AnalogInput eTrackerEncoder;

    public static class Params {
        // PIDS Values
        public double fineTunePower = 0.75;
        public double kP_Up = 0.05;//FIXME
        public double kI_Up = 0.00; //FIXME
        public double kD_Up = 0.000;//FIXME
        public double kS = 0;

<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/subsystem/Extension.java
        public double FIND_BLOCK_POWER = 0.75;

        public int TOLERANCE = 5;
=======
        public int TOLERANCE = 10;
>>>>>>> e612d5cfa6f72653529460ae87c305fc8d7beac6:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/subsystem/ExtensionTele.java

        public static final int EXTENSION_MAX = 500;
        public static final int EXTENSION_MIN = 0;
        public int EXTENSION_CUSTOM = 30;
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

<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/subsystem/Extension.java
    public static Params PARAMS = new Params();
=======
    public static ExtensionTele.Params PARAMS = new ExtensionTele.Params();
>>>>>>> e612d5cfa6f72653529460ae87c305fc8d7beac6:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/subsystem/ExtensionTele.java

    private Gamepad gamepad1;

    // constructor for Extension class
    public ExtensionTele(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
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
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/subsystem/Extension.java
                //setTarget(target);
                //setMotorPower(getControlPower());
                extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(extension.getCurrentPosition() > Params.EXTENSION_MAX)
                     setMotorPower(-0.2);
                else
                    extension.setPower(targetFindBlockPower);
                telemetry.addData("extension power", extension.getPower());
=======
                /*
                if(target > extension.getCurrentPosition() + PARAMS.TOLERANCE)
                    extension.setPower(PARAMS.fineTunePower);
                else if(target < extension.getCurrentPosition() - PARAMS.TOLERANCE)
                    extension.setPower(-PARAMS.fineTunePower);
                else
                    extension.setPower(0);

                 */

                if(gamepad1.dpad_up)
                    extension.setPower(PARAMS.fineTunePower);
                else if(gamepad1.dpad_down)
                    extension.setPower(-PARAMS.fineTunePower);
                else
                    extension.setPower(0);

                //setTarget(target);
                //setMotorPower(getControlPower());
>>>>>>> e612d5cfa6f72653529460ae87c305fc8d7beac6:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/subsystem/ExtensionTele.java
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
