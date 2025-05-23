package org.firstinspires.ftc.teamcode.OpMode;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;
import org.firstinspires.ftc.teamcode.Systems.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="ShowcaseOpMode")
public class ShowcaseOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, true);
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        LynxModule expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");

        ArrayList<Blinker.Step> pattern = new ArrayList<Blinker.Step>();
        pattern.add(new Blinker.Step(Color.RED, 150, TimeUnit.MILLISECONDS));
        pattern.add(new Blinker.Step(Color.YELLOW, 150, TimeUnit.MILLISECONDS));
        pattern.add(new Blinker.Step(Color.GREEN, 150, TimeUnit.MILLISECONDS));

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        double targetHorizontalPosition = 0;

        waitForStart();

        controlHub.setPattern(pattern);
        int loops = 0;

        while (opModeIsActive()) {
            controlHub.clearBulkCache();
            expansionHub.clearBulkCache();
            loops++;
            robot.setOpModeDrivePowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0);

            if (gamepad1.left_bumper) {
                robot.startSpittingSample();
            }
            else if (gamepad1.right_bumper) {
                robot.startTakingSampleIn();
            }
            else {
                robot.stopSampleInteraction();
            }

            if (gamepad1.triangle) {
                robot.setVerticalElevatorsHeight(Elevators.VerticalState.VERTICAL_OPMODE_HIGH.state);
            }
            else if (gamepad1.square) {
                robot.setVerticalElevatorsHeight(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH.state);
            }
            else if (gamepad1.cross) {
                robot.setVerticalElevatorsHeight(Elevators.VerticalState.VERTICAL_PICKUP.state);
            }
            else if (gamepad1.circle) {
                robot.setVerticalElevatorsHeight(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP.state);
            }

            targetHorizontalPosition += (gamepad1.right_trigger - gamepad1.left_trigger)/25.0;
            targetHorizontalPosition = Math.min(1.0, targetHorizontalPosition);
            targetHorizontalPosition = Math.max(0.0, targetHorizontalPosition);

            robot.setHorizontalElevatorsPosition(targetHorizontalPosition);

            if (gamepad1.dpad_down) {
                robot.setClawPosition(DifferentialClaws.ClawPositionState.MIN.state);
            }
            else if (gamepad1.dpad_up) {
                robot.setClawPosition(DifferentialClaws.ClawPositionState.MAX.state);
            }
            else if (gamepad1.dpad_left) {
                robot.setClawPosition(DifferentialClaws.ClawPositionState.SPIT_STATE.state);
            }
            else if (gamepad1.dpad_right) {
                robot.setClawPosition(DifferentialClaws.ClawPositionState.TAKE_SPECIMEN.state);
            }

            robot.update();

            telemetry.addData("loops per second avg", loops/getRuntime());
            telemetry.update();
        }
    }
}
