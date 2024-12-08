package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Lift implements Component {
    public static class Params {
        ;
        public double liftKp = 0.03;
        public double liftKi = 0.0;
        public double liftKd = 0.0;
        public double liftKs = 0.0;

        public int BASE_HEIGHT = 25;
        public int DECONFLICT_HEIGHT = 200;
        public int GRAB_HEIGHT = 17;
        public int LOW_BASKET_HEIGHT = 650;
        public int HIGH_BASKET_HEIGHT = 1100;
        public int SPECIMEN_LEVEL_HEIGHT = 60;
        public int LIFT_SPECIMEN_PRE_DEPOSIT_HEIGHT = 200;
        public int LIFT_SPECIMEN_HIGH_BAR_HEIGHT = 800;
        public int HIGH_BAR_HEIGHT = 640;
        public int HIGHBAR_PRE_HEIGHT = 250;
        public int TOLERANCE = 20;
        public double MAX_POWER_UP = 0.2;
        public double MAX_POWER_DOWN = -0.25;
        public long RESET_TIME = 500;
    }

    PIDController liftController;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public static Params PARAMS = new Params();
    public DcMotorEx liftMotor;
    public LiftState liftState;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        liftController = new PIDController(PARAMS.liftKp, PARAMS.liftKi, PARAMS.liftKd, telemetry);
        liftController.setInputBounds(0, 1500);
        liftController.setOutputBounds(-0.1, 0.99);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        liftState = LiftState.DECONFLICT;
        ///liftState = LiftState.SPECIMEN_LEVEL;
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        SPECIMEN_LEVEL,
        HIGHBAR_PRE_HEIGHT,
    }

    public void setMotorPower(double power) {
        liftMotor.setPower(power);
    }

    @Override
    public void reset() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setTarget(double target) {
        liftController.setTarget(target);
    }

    private void liftDown() {
        setTarget(0);
        setMotorPower(PARAMS.MAX_POWER_DOWN);
    }

    private void selectState() {
        switch (liftState) {
            case RESET:
                liftDown();
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
                break;

            case HIGHBAR_PRE_HEIGHT:
                setTarget(PARAMS.HIGHBAR_PRE_HEIGHT);
                break;
        }
    }

    public boolean greaterHighBar() {
        return liftMotor.getCurrentPosition() > PARAMS.HIGH_BAR_HEIGHT;
    }

    private double getControlPower() {
        double pidPower = -liftController.update(liftMotor.getCurrentPosition());

        return pidPower + feedForwardPower();
    }

    private double feedForwardPower() {
//        double x = liftMotor.getCurrentPosition();
//        double m = -0.0;
//        double b = -0.0;

        return PARAMS.liftKs;
    }

    @Override
    public void update() {
        selectState();
        double power;

        if(liftState == LiftState.HIGH_BAR) {
            power = 1.0;
        } else {
            power = getControlPower();
        }

        setMotorPower(power);

        telemetry.addData("liftController Target", liftController.getTarget());
        telemetry.addData("liftMotor Position", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotor Power", liftMotor.getPower());
        telemetry.addData("Lift State", liftState);

    }

    public void setReset() {
        liftState = LiftState.RESET;
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
        return Math.abs(liftMotor.getCurrentPosition() - liftController.getTarget()) < PARAMS.TOLERANCE;
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

    public void setHighBarPreHeight(){ liftState = liftState.HIGHBAR_PRE_HEIGHT; }
}