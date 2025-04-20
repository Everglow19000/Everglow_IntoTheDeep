package org.firstinspires.ftc.teamcode.AxonSwerve;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


/*
    -----------------------------------------------------
    | This uses radians cuz they will work nice with RR |
    -----------------------------------------------------
 */


public class EncodedCRServo {
    public enum RunMode {
        RUN_WITH_ENCODER, // Rotates according to the given power and tracks it's position
        RUN_TO_POSITION // Runs to the given position, in degrees
    }
    private CRServo servo;
    private AnalogInput input;
    private double position = 0;
    private double lastAngle;
    private double targetPosition = 0;
    private double inputOffset = 0;

    private double power;
    private static PIDFCoefficients defaultPIDFCoefficients = new PIDFCoefficients(
            0.001,
            0,
            0.001,
            0.1
    );
    private PIDFCoefficients pidfCoefficients;
    private PIDFController pidfController;
    private RunMode runMode = RunMode.RUN_TO_POSITION;

    public EncodedCRServo(CRServo servo, AnalogInput input) {
        this(servo, input, defaultPIDFCoefficients);
    }

    public EncodedCRServo(CRServo servo, AnalogInput input, PIDFCoefficients pidfCoefficients) {
        this.servo = servo;
        this.input = input;
        resetEncoder();
        setPidfCoefficients(pidfCoefficients);
    }

    public void setPidfCoefficients(PIDFCoefficients pidfCoefficients) {
        setPidfCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);
    }

    public void setPidfCoefficients(double p, double i, double d, double f) {
        pidfCoefficients = new PIDFCoefficients(p, i, d, f);
        pidfController.setPIDF(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);
    }

    private double getCurrentAngle() {
        return ((input.getVoltage()/input.getMaxVoltage())*Math.PI*2) - inputOffset;
    }

    private void calculatePosition() {
        double currAngle = getCurrentAngle();
        double angleDifference = currAngle - lastAngle;

        if (angleDifference > Math.PI) {
            angleDifference -= Math.PI*2;
        }
        else if (angleDifference < -Math.PI) {
            angleDifference += Math.PI*2;
        }
        position += angleDifference;
        lastAngle = currAngle;
    }

    public double getCurrentPosition() {
        return position;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setRunMode(RunMode runMode) {
        this.runMode = runMode;
    }

    public RunMode getRunMode() {
        return runMode;
    }

    public void resetEncoder() {
        lastAngle = 0;
        inputOffset = getCurrentAngle();
        position = 0;
        targetPosition = 0;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        pidfController.setSetPoint(targetPosition);
    }

    public void setPower(double power) {
        this.power = power;
    }
    public double getPower() {
        return power;
    }

    public double getPIDFPower() {
        pidfController.setSetPoint(targetPosition);
        return pidfController.calculate(position);
    }

    public void update() {
        calculatePosition();

        if (runMode == RunMode.RUN_TO_POSITION) {
            servo.setPower(getPIDFPower());
        }
        else {
            servo.setPower(power);
        }
    }
}
