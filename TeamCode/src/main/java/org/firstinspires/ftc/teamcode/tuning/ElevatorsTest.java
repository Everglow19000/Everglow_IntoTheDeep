package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Config
@TeleOp(name = "Elevator Test", group = "Tests")

public class ElevatorsTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Elevators elevators = Elevators.getInstance(this);
        boolean flagElevatorVerticalDpadUp = true;
        boolean flagElevatorVerticalDpadDown = true;

        waitForStart();
        double pos = 0;
        while (opModeIsActive()){
            telemetry.addData("vert elevators pos:", elevators.getVerticalCurrentPosition());
            telemetry.addData("vert left vel:", elevators.getLeftVelocity());
            telemetry.addData("vert right vel:", elevators.getRightVelocity());
            telemetry.addData("rightHor pos", elevators.getRightHorPos());
            telemetry.addData("leftHor pos", elevators.getLeftHorPos());
            telemetry.update();
            if(gamepad1.cross){
                pos += 0.008;
                pos = Math.min(1, pos);
            }

            if(gamepad1.square){
                pos -= 0.008;
                pos = Math.max(0, pos);
            }

//            if (gamepad2.square){
//                elevators.setVerticalDestination(0);
//                elevators.setVerticalPower(0);
//            }
//
//            if(gamepad2.dpad_up && flagElevatorVerticalDpadUp){
//                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_LOW.state);
//            }
//            flagElevatorVerticalDpadUp = !gamepad2.dpad_up;
//
//            if(gamepad2.dpad_down && flagElevatorVerticalDpadDown) {
//                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_PICKUP.state);
//            }
//            flagElevatorVerticalDpadDown = !gamepad2.dpad_down;

            elevators.setHorizontalDestination(pos);
        }
    }
}
