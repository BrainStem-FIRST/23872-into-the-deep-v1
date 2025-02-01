package org.firstinspires.ftc.teamcode.auto.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Lift implements Component {
    public static class Params {
        ;
        public double liftKp = 0.02;
        public double liftKi = 0.01;
        public double liftKd = 0.0001;

        public int BASE_HEIGHT = 25;
        public int DECONFLICT_HEIGHT = 200;
        public int GRAB_HEIGHT = 20;
        public int LOW_BASKET_HEIGHT = 600;
        public int HIGH_BASKET_HEIGHT = 1000;
        public int SPECIMEN_LEVEL_HEIGHT = 80;
        public int LIFT_SPECIMEN_PRE_DEPOSIT_HEIGHT = 200;
        public int LIFT_SPECIMEN_HIGH_BAR_HEIGHT = 500;
        public int HIGH_BAR_HEIGHT = 800;
        public int TOLERANCE = 30;
    }

    PIDController liftController;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public static Params PARAMS = new Params();
    public DcMotorEx liftMotor;
    public LiftState liftState;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        liftController = new PIDController(PARAMS.liftKp, PARAMS.liftKi, PARAMS.liftKd);
        liftController.setInputBounds(0, 4000);
        liftController.setOutputBounds(-0.1, 0.99);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        liftState = LiftState.DECONFLICT;
        ///liftState = LiftState.SPECIMEN_LEVEL;
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public enum LiftState {
        BASE,
        DECONFLICT,
        LOW_BASKET,
        HIGH_BASKET,
        RESET,
        GRAB,
        LIFT_SPECIMEN_PRE_DEPOSIT,
        LIFT_SPECIMEN_HIGH_BAR,
        HIGH_BAR,
        SPECIMEN_LEVEL
    }

    public void setMotorPower(double power) {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
    }

    @Override
    public void reset() {
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setTarget(int target) {
        liftMotor.setTargetPosition(target);
    }

    private void selectState() {
        switch (liftState) {
            case RESET:
                reset();
                break;

            case GRAB:
                setTarget(PARAMS.GRAB_HEIGHT);
                break;

            case BASE:
                setTarget(PARAMS.BASE_HEIGHT);
                break;

            case DECONFLICT:
                setTarget(PARAMS.DECONFLICT_HEIGHT);
                break;

            case LOW_BASKET:
                setTarget(PARAMS.LOW_BASKET_HEIGHT);
                break;

            case HIGH_BASKET:
                setTarget(PARAMS.HIGH_BASKET_HEIGHT);
                break;

            case SPECIMEN_LEVEL:
                setTarget(PARAMS.SPECIMEN_LEVEL_HEIGHT);
                break;

            case LIFT_SPECIMEN_PRE_DEPOSIT:
                setTarget(PARAMS.LIFT_SPECIMEN_PRE_DEPOSIT_HEIGHT);
                break;


            case LIFT_SPECIMEN_HIGH_BAR:
                setTarget(PARAMS.LIFT_SPECIMEN_HIGH_BAR_HEIGHT);
                break;

            case HIGH_BAR:
                setTarget(PARAMS.HIGH_BAR_HEIGHT);
                break;
        }
    }

    private double getControlPower() {
        double pidPower = -liftController.update(liftMotor.getCurrentPosition());

        return pidPower + feedForwardPower();
    }

    private double feedForwardPower() {
        double x = liftMotor.getCurrentPosition();
        double m = -0.0;
        double b = -0.0;

        return m * x + b;
    }

    @Override
    public void update() {
        selectState();
        setMotorPower(1.0);
    }

    public void setBase() {
        liftState = LiftState.BASE;
    }

    public void setGrab() {
        liftState = LiftState.GRAB;
    }

    public void setDeconflict() {
        liftState = LiftState.DECONFLICT;
    }

    public boolean inTolerance() {
        return Math.abs(liftMotor.getCurrentPosition() - liftMotor.getTargetPosition()) < PARAMS.TOLERANCE;
    }

    public boolean inTightTolerance() {
        return Math.abs(liftMotor.getCurrentPosition() - liftMotor.getTargetPosition()) < (PARAMS.TOLERANCE - 10);
    }

    public void setLowBasket() {
        liftState = LiftState.LOW_BASKET;
    }

    public void setHighBasket() {
        liftState = LiftState.HIGH_BASKET;
    }

    @Override
    public String test() {
        return "Complete";
    }

    public void setSpecimenLevel() {
        liftState = LiftState.SPECIMEN_LEVEL;
    }

    public void setSpecimenPreDeposit() {
        liftState = liftState.LIFT_SPECIMEN_PRE_DEPOSIT;
    }

    public void setLiftSpecimenHighBar() {
        liftState = liftState.LIFT_SPECIMEN_HIGH_BAR;
    }

    public void setHighBar() {
        liftState = liftState.HIGH_BAR;
    }

    public class GotoHighBasket implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftState = LiftState.HIGH_BASKET;
                initialized = true;
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(1.0);
            }

//            if (liftMotor.getCurrentPosition() > 1000) {
//                update();
//            }

            packet.put("Lift Pos", liftMotor.getCurrentPosition());

            return liftMotor.getCurrentPosition() < 500;
        }
    }

    public Action gotoHighBasket() {
        return new GotoHighBasket();
    }

    public class GotoGrab implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftState = LiftState.GRAB;
                initialized = true;
            }

            update();
            return !inTightTolerance();
        }
    }

    public Action gotoGrab() {
        return new GotoGrab();
    }

    public class GotoDeconflict implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                liftState = LiftState.DECONFLICT;
                initialized = true;
            }

            update();
            return !inTightTolerance();
        }
    }

    public Action gotoDeconflict() {
        return new GotoDeconflict();
    }
}