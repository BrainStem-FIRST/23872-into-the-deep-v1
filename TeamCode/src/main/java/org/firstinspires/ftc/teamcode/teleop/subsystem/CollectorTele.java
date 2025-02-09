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
        // checking for a current spike
        if (collectorMotor.getCurrent(CurrentUnit.MILLIAMPS) > currentThreshold) {
            currentCounter += 1;
            extakeExtraTimer.reset();
            // resetting current count timer once pass a safety extake threshold
        } else {
            if(extakeExtraTimer.seconds() > extakeExtraTime)
                currentCounter = 0;

            // extaking once current has spike for a validated amount of frames
        }
        if (currentCounter > 10) {
            collectorMotor.setPower(outtakePower);
        }
        // collecting if there is no current spike
        else {
            collectorMotor.setPower(0.99);
        }
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
