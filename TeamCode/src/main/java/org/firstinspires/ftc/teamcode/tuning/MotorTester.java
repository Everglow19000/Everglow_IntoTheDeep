package org.firstinspires.ftc.teamcode.tuning;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor test")
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        GamepadButton grabButton = new GamepadButton(
                gamepadEx, GamepadKeys.Button.A
        );
        GamepadButton releaseButton = new GamepadButton(
                gamepadEx, GamepadKeys.Button.B
        );

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("is x pressed?", gamepad1.x);
            telemetry.update();
            if (gamepad1.x){
                motor.setPower(0.8);
            } else {
                motor.setPower(0);
            }
        }
    }
}
