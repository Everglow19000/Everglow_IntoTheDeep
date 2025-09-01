package org.firstinspires.ftc.teamcode.AxonSwerve;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SwerveModule {
    static double maxMetersPerSecond = 2; //TODO: CHECK THIS AND MAKE GOOD
    EncodedCRServo angularMotor;
    DcMotorEx driveMotor;

    //TODO: TUNE THESE SO IT WORK GOOD :)
    private static PIDFCoefficients angularMotorPIDF = new PIDFCoefficients(
            0.001,
            0,
            0.001,
            0.1
    );


    public SwerveModule(CRServo servo, AnalogInput input, DcMotorEx driveMotor) {
        this(new EncodedCRServo(servo, input, angularMotorPIDF), driveMotor);
    }

    public SwerveModule(EncodedCRServo angularMotor, DcMotorEx driveMotor) {
        this.angularMotor = angularMotor;
        angularMotor.setPidfCoefficients(angularMotorPIDF);
        angularMotor.resetEncoder();
        this.driveMotor = driveMotor;
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setAngle(double angle) {
        angularMotor.setTargetPosition(angle%(Math.PI*2));
    }

    public void setDrivePower(double power) {
        driveMotor.setPower(power);
    }

    public void setState(SwerveModuleState state) {
        setAngle(state.angle.getRadians());
        setDrivePower(state.speedMetersPerSecond);
    }


    // must be called every iteration
    public void update() {
        angularMotor.update();
    }
}
