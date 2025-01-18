package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@TeleOp(name = "FirstOpMode")
public class FirstOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Elevators elevators = new Elevators(this);
        elevators.setVerticalPower(0.0);
        DifferentialClaws claws = new DifferentialClaws(this);

        waitForStart();
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        LynxModule expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");

        elevators.motorSetHorizontalPower(0.8);

        double epsilon = 0.4;
        double joystickTolerance = 0.05;
        boolean flagElevatorVerticalDpadDown = true;
        boolean flagElevatorVerticalDpadLeft = true;
        boolean flagElevatorVerticalDpadUp = true;
        boolean flagElevatorVerticalDpadRight = true;

        boolean flagElevatorHorizontalX = true;
        boolean flagElevatorHorizontalTriangle = true;
        boolean flagElevatorHorizontalCircle = true;
        boolean flagElevatorHorizontalSquare = true;

        boolean flagClawTakeIn = true;
        boolean ClawState = true;
        boolean flagClawSpit = true;

        double horElevatorPosition = 0;

        double AnalogueExtensionVertical;
        double VerticalAnalogueFactor = 1;

        double HorizontalAnalogueFactor = 1;
        double AnalogueExtensionHorizontal;

        while (opModeIsActive()) {
            //driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();


            if (gamepad2.right_trigger >= 0.35) { //split
                claws.rotateWheels(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger >= 0.4) {
                claws.rotateWheels(-1);
            }
            else {
                Actions.runBlocking(claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.OFF));
                claws.rotateArm(-gamepad2.left_stick_y*0.75);
            }

            if (Math.abs(gamepad2.right_stick_y) > joystickTolerance) {
                if(horElevatorPosition < 0){
                    horElevatorPosition = 0;
                }else if(horElevatorPosition > 3700){
                    horElevatorPosition =  3700;
                }
                horElevatorPosition += -gamepad2.right_stick_y*40*1.5;
                elevators.motorSetHorizontalDestination((int)(horElevatorPosition));
            }
            telemetry.addData("Right Stick y: ", gamepad2.right_stick_y);
            telemetry.addData("precieved hor position: ", horElevatorPosition);
            telemetry.addData("hor position: ", elevators.motorGetHorizontalPosition());
            telemetry.addData("Control Hub auxillary volts: ", controlHub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
            telemetry.addData("Expansion Hub auxillary volts: ", expansionHub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
            telemetry.addData("Control Hub used volts: ", controlHub.getInputVoltage(VoltageUnit.VOLTS));
            telemetry.addData("Expansion Hub used volts: ", expansionHub.getInputVoltage(VoltageUnit.VOLTS));
            telemetry.update();



            if(gamepad2.dpad_down && flagElevatorVerticalDpadDown) {
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_PICKUP.state);
            }
            flagElevatorVerticalDpadDown = !gamepad2.dpad_down;

            if(gamepad2.dpad_left && flagElevatorVerticalDpadLeft){
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_HURDLE.state);
            }
            flagElevatorVerticalDpadLeft = !gamepad2.dpad_left;

            if(gamepad2.dpad_up && flagElevatorVerticalDpadUp){
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_LOW.state);
            }
            flagElevatorVerticalDpadUp = !gamepad2.dpad_up;

            if(gamepad2.dpad_right && flagElevatorVerticalDpadRight){
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_HIGH.state);
            }
            flagElevatorVerticalDpadRight = !gamepad2.dpad_right;

            telemetry.addData("vert pos:", elevators.getVerticalCurrentPosition());

            // Read line 105
//            AnalogueExtensionVertical = -gamepad2.left_stick_y;
//            elevators.setVerticalDestination((int)(elevators.getVerticalDestination() + AnalogueExtensionVertical * VerticalAnalogueFactor));
//
            if(gamepad2.cross && flagElevatorHorizontalX) {
                elevators.motorSetHorizontalDestination(Elevators.MotorHorizontalState.HORIZONTAL_EXTENDED.state);
                horElevatorPosition = Elevators.MotorHorizontalState.HORIZONTAL_EXTENDED.state;
            }
            flagElevatorHorizontalX = !gamepad2.cross;
//
            if(gamepad2.triangle && flagElevatorHorizontalTriangle){
                elevators.motorSetHorizontalDestination(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED.state);
                horElevatorPosition = Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED.state;
            }
            flagElevatorHorizontalTriangle = !gamepad2.triangle;
//
//            if(gamepad2.square && flagElevatorHorizontalSquare){
//                elevators.setHorizontalPosition(Elevators.HorizontalState.HORIZONTAL_DROP.state);
//            }
//            flagElevatorHorizontalSquare = !gamepad2.square;
//
//
            if(gamepad2.circle && flagElevatorHorizontalCircle){
                elevators.motorSetHorizontalDestination(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY.state);
                horElevatorPosition = Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY.state;
            }
            flagElevatorHorizontalCircle = !gamepad2.circle;
            elevators.updateVert();

            if(elevators.getVertDestination() == 0 && gamepad2.square){
                elevators.resetVert();
            }

//
//            Currently Broken, might cause damage to robot
//            AnalogueExtensionHorizontal = -gamepad2.right_stick_x;
//            elevators.setHorizontalPosition(elevators.getHorizontalState() + AnalogueExtensionHorizontal * HorizontalAnalogueFactor);
//
//            telemetry.addData("Epsilon: ", epsilon);
//            telemetry.addData("Right trigger thing: ", gamepad2.right_trigger);
//            telemetry.addData("flag claw take in: ", flagClawTakeIn);
//
//            // take in trigger
//            if(gamepad2.right_trigger > epsilon  && flagClawTakeIn){
//                telemetry.addLine("Should be taking in ");
//                claw.setState(Claws.ClawState.TAKE_IN);
//            }
//            flagClawTakeIn = !(gamepad2.right_trigger > epsilon);
//
//            // spit trigger
//            if(gamepad2.right_bumper && flagClawSpit){
//                claw.setState(Claws.ClawState.SPIT);
//            }
//            flagClawSpit = !gamepad2.right_bumper;
//
//            // claw off
//            if(!gamepad2.right_bumper && !(gamepad2.right_trigger > epsilon)){
//                claw.setState(Claws.ClawState.OFF);
//            }
//
//
//            telemetry.addData("position:", drive.pose);
//            telemetry.addData("right_bumper:", gamepad2.right_trigger);
//            telemetry.addData("right_trigger:", gamepad2.right_bumper);
//            telemetry.update();
        }

    }
}