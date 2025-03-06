/*
--------------------------------------------------------------------------
| This is going to work by internally keeping each servo's rotation in   |
| degrees (0 is starting) and possibly the entire claw's position.       |
| Actions must be used so that the servo actually reaches its position.  |
--------------------------------------------------------------------------
*/

package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.Token.TokenAction;

public class DifferentialClaws {

    static CRServo leftClawServo;
    static CRServo rightClawServo;
    static AnalogInput clawInput1;
    static AnalogInput clawInput2;

    private static DifferentialClaws instance = null;

    public static final double maxPoint = 68+46;

    double armPosition = 0;
    double lastPosRequest = 0;

    boolean isGoingDown = false;

    // tracks from -∞ - ∞ the rotation of each motor.
    //double leftClawServoRotation = 0;
    //double rightClawServoRotation = 0;

    private double leftClawOldPos;
    private double rightClawOldPos;

    private final double leftClawStart;
    private final double rightClawStart;

    private double trueLeftRotation = 0;
    private double trueRightRotation = 0;

    public static PIDController controller;

    public final double p = 0.009,//.008,
            i = 0,
            d = 0.0002;//.0001;
    public double f = 0.12;//0.08;

    private double target = 0;

    private ClawPowerState wheelRotationState;

    private double armStartingPosition;

    public class ClawMovementAction extends TokenAction {

        double destination;
        int timeTillFinish;
        long startTime;
        public ClawMovementAction(double destination, int timeTillFinish) {
            this.timeTillFinish =timeTillFinish;
            this.destination = destination;
            isDone = this::isWaitEnough;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setArmTargetPosition(destination);
            updateLeftClawServoRotation();
            updateRightClawServoRotation();
            rotateArm(getPIDArmPower());

            if(!isInitialized){
                startTime = System.currentTimeMillis();
                isInitialized = true;
            }
            return !isWaitEnough();
        }
        private boolean isWaitEnough(){
            return System.currentTimeMillis() - startTime >= timeTillFinish;
        }
    }
    // receives the time in milliseconds until the action is considered finished
    public class ClawSampleInteractionAction extends TokenAction {
        private final double wantedPower;
        private double timeUntilFinished;
        private double startTime;
        private ColorSensorSystem colorSensorSystem = null;
        boolean isIn = false;
        boolean isToInsert;

        private ClawSampleInteractionAction(ClawPowerState state, double timeToStop) {
            assert timeToStop >= 0;
            this.wantedPower = state.state;
            this.timeUntilFinished = timeToStop;
            isDone = this::isFinished;
        }
        public ClawSampleInteractionAction(ClawPowerState state, ColorSensorSystem colorSensorSystem, boolean insert) {
            this(state, 2500);
            isToInsert = insert;
            this.colorSensorSystem = colorSensorSystem;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            updateRightClawServoRotation();
            updateLeftClawServoRotation();
            if (!isInitialized) {
                leftClawServo.setPower(wantedPower);
                rightClawServo.setPower(-wantedPower);
                startTime = System.currentTimeMillis();
                isInitialized = true;
            }

            if(colorSensorSystem != null && colorSensorSystem.isSpecimenIn() == isToInsert && !isIn)
            {
                startTime = System.currentTimeMillis();
                if(isToInsert)
                    timeUntilFinished = 450;
                else
                    timeUntilFinished = 700;
                isIn = true;
            }

            return !isFinished();
        }

        private boolean isFinished(){
            return (System.currentTimeMillis() - startTime > timeUntilFinished);
        }
    }

    public class HoldClawAndDropSampleAction implements Action {
        private final double holdingPower = -0.15;
        private final double timeToHold; // time to hold the arm in place until dropping the sample in ms
        private final double timeUntilDropDone; // time until the drop is considered done in ms
        private boolean isHoldInit = false;
        private boolean isDropInit = false;
        private double startTime = 0;


        public HoldClawAndDropSampleAction(double timeToHold, double timeUntilDropDone) {
            this.timeToHold = timeToHold;
            this.timeUntilDropDone = timeUntilDropDone;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isHoldInit) {
                startTime = System.currentTimeMillis();
                rightClawServo.setPower(holdingPower);
                leftClawServo.setPower(holdingPower);
                isHoldInit = true;
            }

            if (!isDropInit && isHoldInit && System.currentTimeMillis() - startTime > timeToHold) {
                rightClawServo.setPower(holdingPower -0.15);
                leftClawServo.setPower(holdingPower +0.15);
                isDropInit = true;
            }

            if (isDropInit && isHoldInit && System.currentTimeMillis() - startTime > timeUntilDropDone) {
                rotateWheels(0);
                return false;
            }
            return true;
        }
    }

    private DifferentialClaws(OpMode opMode) {
        leftClawServo = opMode.hardwareMap.get(CRServo.class, "leftClawServo");
        rightClawServo = opMode.hardwareMap.get(CRServo.class, "rightClawServo");
        clawInput1 = opMode.hardwareMap.get(AnalogInput.class, "clawInput1");
        clawInput2 = opMode.hardwareMap.get(AnalogInput.class, "clawInput2");

        leftClawServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightClawServo.setDirection(DcMotorSimple.Direction.FORWARD);

        rotateArm(0.01);
        rotateArm(0);

        controller = new PIDController(p, i, d);
        updateLeftClawServoRotation();
        updateRightClawServoRotation();
        rightClawStart = trueRightRotation;
        leftClawStart = trueLeftRotation;
        leftClawOldPos = leftClawStart;
        rightClawOldPos = rightClawStart;
        armStartingPosition = getArmPosition() + maxPoint;

    }

    public static DifferentialClaws getInstance(OpMode opMode) {
        if (instance == null) {
            instance = new DifferentialClaws(opMode);
        }
        else {
            instance.rotateArm(0.01);
            instance.rotateArm(0);

            instance.setCorrectDirections();
        }

        return instance;
    }

    public enum ClawPositionState {
        MIN(0.0),
        MID(maxPoint/2),
        SPIT_STATE(72),
        HANG_SPECIMEN(53),
        MAX(maxPoint);

        public final double state;
        ClawPositionState(double state) {this.state = state;}
    }

    public enum ClawPowerState {
        TAKE_IN(-1),
        OFF(0.08),
        SPIT(0.25);

        public final double state;

        ClawPowerState(double state) {this.state = state;}
    }

    public void setCorrectDirections() {
        leftClawServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightClawServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public double getArmPosition() {
        double leftDiff = trueLeftRotation - leftClawStart;
        double rightDiff = trueRightRotation - rightClawStart;

        return Math.abs(((-rightDiff+leftDiff)/2)*(20.0/43.0));
    }

    public static double getRotationOfInput(AnalogInput input) {
        return (input.getVoltage() / input.getMaxVoltage()) * 360;
    }

    public void updateLeftClawServoRotation() {
        double currentRotation = getRotationOfInput(clawInput1);
        double diff = currentRotation - leftClawOldPos;

        double newRotationEstimate = 180;
        if(Math.abs(diff) > newRotationEstimate){
            //new rotation occur
            if(diff < 0)
                diff += 360; //add rotation
            else
                diff -= 360; //minus rotation
        }

        leftClawOldPos = currentRotation;
        trueLeftRotation += diff;
    }

    public void updateRightClawServoRotation() {
        double currentRotation = getRotationOfInput(clawInput2);
        double diff = currentRotation - rightClawOldPos;

        double newRotationEstimate = 180;
        if(Math.abs(diff) > newRotationEstimate){
            //new rotation occur
            if(diff < 0)
                diff += 360; //add rotation
            else
                diff -= 360; //minus rotation
        }

        rightClawOldPos = currentRotation;
        trueRightRotation += diff;
    }
    public double getActualArmRotation() {
        return Math.max(getArmPosition() - armStartingPosition, armStartingPosition - getArmPosition());
    }
    public void setF(double f){
        this.f = f;
    }
    public double getPIDArmPower(){
        int armPos = (int)(getActualArmRotation());
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((armPos/maxPoint)*180. - 30.)) * f;

        return -(pid + ff);
    }
    public void rotateArm(double power){
        leftClawServo.setPower(power);
        rightClawServo.setPower(power);
    }


    public void rotateWheels(double state) {
        leftClawServo.setPower(state);
        rightClawServo.setPower(-state);
    }

    public void rotateWheels(ClawPowerState state) {
        wheelRotationState = state;
        leftClawServo.setPower(state.state);
        rightClawServo.setPower(-state.state);
    }

    public void setArmTargetPosition(double pos){
        target = pos;
    }

    public double getArmTargetPosition(){
        return target;
    }
    public double[] getServoVirtualPosition(){
        return new double[] {trueLeftRotation, trueRightRotation};
    }
    public void setPower(double p1, double p2){
        double sum = p1+p2;
        p1 /= sum;
        p2 /= sum;

        leftClawServo.setPower(p1);
        rightClawServo.setPower(p2);
    }

    public double getleftClawServoRotation() {
        return getRotationOfInput(clawInput1);
    }

    public double getrightClawServoRotation() {
        return getRotationOfInput(clawInput2);
    }

    public double getClawRotation() {
        return armPosition;
    }

    public ClawPowerState getRotationState() {
        return wheelRotationState;
    }


    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state, double timeUntilFinished) {
        return new ClawSampleInteractionAction(state, timeUntilFinished);
    }

    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state, ColorSensorSystem colorSensorSystem) {
        return new ClawSampleInteractionAction(state, colorSensorSystem, state == ClawPowerState.TAKE_IN);
    }

    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state) {
        return new ClawSampleInteractionAction(state, 0);
    }

    // gets in degrees
//    public ClawMovementAction addClawMovementAction(double armPosition, Telemetry telemetry) {
//        return new ClawMovementAction(armPosition, telemetry);
//    }

    // gets in degrees, adds the given to the current position
    public ClawMovementAction addClawMovementAction(double armPosition) {
        double out_val = this.armPosition + armPosition;
        return clawMovementAction(out_val, 750);
    }

    //gets in degrees, sets the claw's position to the given position
    public ClawMovementAction clawMovementAction(double dest, int timeTillFinish) {
        //double diff = armPosition - this.armPosition;
        //ClawMovementAction action =;
        //this.armPosition = armPosition;
        return new ClawMovementAction(dest, timeTillFinish);
    }

    public HoldClawAndDropSampleAction test(double timeToHold, double timeToDrop) {
        return new HoldClawAndDropSampleAction(timeToHold, timeToDrop);
    }

//    public ClawMovementAction clawMovementAction(double dest) {
//        return new ClawMovementAction(dest);
//    }
}
