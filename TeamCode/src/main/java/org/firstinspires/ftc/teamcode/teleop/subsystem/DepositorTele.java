package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingServo;

@Config
public class DepositorTele implements ComponentTele {
    public static class Params {
        public double depositorForwardPosition = 0.85;
        public double depositorBackwardPosition = 0.350;
        public double depositorDownPosition = 0.075;
        public double depositorUpPosition = 0.65;
        public double depositorHighBasket = 0.5;
        public double depositorNeutralPosition = 0.5;
        public double gripperClosedPosition = 0.99;
        public double gripperOpenedPosition = 0.065;
        public double gripperLowerPWM = 400;
        public double gripperUpperPWM = 1800;

        public final static int GRIPPER_OPEN_TIME_MS = 250;
        public final static int GRIPPER_CLOSE_TIME_MS = 250;

        public final static int DEPOSITOR_FORWARD_TIME_MS = 250;
        public final static int DEPOSITOR_BACK_TIME_MS = 250;
        public final static int DEPOSITOR_UP_TIME_MS = 400;
        public final static int DEPOSITOR_DOWN_TIME_MS = 400;
        public final static int DEPOSITOR_HIGH_BASKET_TIME_MS = 400;
    }

    Telemetry telemetry;
    HardwareMap hardwareMap;
    public static Params PARAMS = new Params();
    CachingServo rotationServo;
    CachingServo gripperServo;


    public DepositorState depositorState;
    public GripperState gripperState;

    public DepositorTele(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        rotationServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "depositorRotationServo"));
        gripperServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "gripperServo"));
        gripperServo.setPwmRange(new PwmControl.PwmRange(PARAMS.gripperLowerPWM, PARAMS.gripperUpperPWM));
        depositorState = DepositorState.DEPOSITOR_DOWN;
        gripperState = GripperState.GRIPPER_OPEN;
    }

    public enum GripperState {
        GRIPPER_CLOSED,
        GRIPPER_OPEN
    }

    public enum DepositorState {
        DEPOSITOR_FORWARD,
        DEPOSITOR_BACKWARD,
        DEPOSITOR_NEUTRAL,
        DEPOSITOR_DOWN,
        DEPOSITOR_UP,
        DEPOSITOR_HIGH_BASKET,
    }

    @Override
    public void reset() {
    }

    @Override
    public String test() {
        return "true";
    }

    private void selectGripperState() {
        switch (gripperState) {
            case GRIPPER_CLOSED:
                closeGripper();
                break;

            case GRIPPER_OPEN:
                openGripper();
                break;

        }
    }

    private void selectDepositorState() {
        switch (depositorState) {
            case DEPOSITOR_FORWARD:
                moveDepositorForward();
                break;

            case DEPOSITOR_DOWN:
                moveDepositorDown();
                break;

            case DEPOSITOR_UP:
                moveDepositorUp();
                break;

            case DEPOSITOR_BACKWARD:
                moveDepositorBackward();
                break;

            case DEPOSITOR_NEUTRAL:
                moveDepositorNeutral();
                break;

            case DEPOSITOR_HIGH_BASKET:
                moveDepositorHighBasket();
                break;
        }
    }

    @Override
    public void update() {
        selectGripperState();
        selectDepositorState();
    }

    private void moveDepositorForward() {
        rotationServo.setPosition(PARAMS.depositorForwardPosition);
    }

    private void moveDepositorBackward() {
        rotationServo.setPosition(PARAMS.depositorBackwardPosition);
    }

    private void moveDepositorDown() {
        rotationServo.setPosition(PARAMS.depositorDownPosition);
    }
    private void moveDepositorUp() {
        rotationServo.setPosition(PARAMS.depositorUpPosition);
    }

    private void moveDepositorNeutral() {
        rotationServo.setPosition(PARAMS.depositorNeutralPosition);
    }

    private void moveDepositorHighBasket() {
        rotationServo.setPosition(PARAMS.depositorHighBasket);
    }

    public void setDepositorForward() {
        depositorState = DepositorState.DEPOSITOR_FORWARD;
    }

    public void setDepositorHighBasket() {
        depositorState = DepositorState.DEPOSITOR_HIGH_BASKET;
    }

    public void setDepositorBackward() {
        depositorState = DepositorState.DEPOSITOR_BACKWARD;
    }

    public void setDepositorDown() {
        depositorState = DepositorState.DEPOSITOR_DOWN;
    }
    public void setDepositorUp() {
        depositorState = DepositorState.DEPOSITOR_UP;
    }

    public void setDepositorNeutral() {
        depositorState = DepositorState.DEPOSITOR_NEUTRAL;
    }

    private void closeGripper() {
        gripperServo.setPosition(PARAMS.gripperClosedPosition);
    }

    private void openGripper() {
        gripperServo.setPosition(PARAMS.gripperOpenedPosition);
    }

    public void setGripperClosed() {
        gripperState = GripperState.GRIPPER_CLOSED;
    }

    public void setGripperOpen() {
        gripperState = GripperState.GRIPPER_OPEN;
    }
    }

