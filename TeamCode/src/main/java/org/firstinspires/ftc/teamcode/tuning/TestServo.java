package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestServo", group = "Tests")
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "HorServo");

        servo.setPosition(0);
        double virtualPos = 0;
        waitForStart();

        while (opModeIsActive()){
            virtualPos -= gamepad1.left_stick_y/500;
            servo.setPosition(virtualPos);
            telemetry.addData("servo pos:", servo.getPosition());
            telemetry.update();

        }
    }
}
