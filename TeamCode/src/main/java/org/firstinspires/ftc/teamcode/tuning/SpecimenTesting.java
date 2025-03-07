package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ActionControl;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Disabled
@TeleOp(name="SpecimenTesting", group="Tests")
public class SpecimenTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = DifferentialClaws.getInstance(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Elevators elevators = Elevators.getInstance(this);
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, false);
        elevators.setVerticalPower(0.0);
        boolean isInitialized = false;
        boolean secondery = false;

        ActionControl control = new ActionControl(elevators, claws, colorSensorSystem, drive, gamepad1, gamepad2);

        waitForStart();
        //LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        //LynxModule expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");

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

        boolean flagRightBumper = true;

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


            if (gamepad2.right_trigger >= 0.4) { //split
                claws.rotateWheels(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger >= 0.4) {
                claws.rotateWheels(-1);
            }
            else {
                claws.rotateArm(DifferentialClaws.ClawPowerState.OFF.state);
                claws.rotateArm(gamepad2.left_stick_y);
            }

            if (Math.abs(gamepad2.right_stick_y) > joystickTolerance) {
                if(horElevatorPosition < 0){
                    horElevatorPosition = 0;
                }else if(horElevatorPosition >= Elevators.HorizontalState.HORIZONTAL_EXTENDED.state){
                    horElevatorPosition =  Elevators.HorizontalState.HORIZONTAL_EXTENDED.state;
                }
                horElevatorPosition += -gamepad2.right_stick_y*40*3;
                elevators.setHorizontalDestination((int)(horElevatorPosition));
            }
            telemetry.addData("Right Stick y: ", gamepad2.right_stick_y);
            telemetry.addData("precieved hor position: ", horElevatorPosition);
//            telemetry.addData("Control Hub auxillary volts: ", controlHub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
//            telemetry.addData("Expansion Hub auxillary volts: ", expansionHub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
//            telemetry.addData("Control Hub used volts: ", controlHub.getInputVoltage(VoltageUnit.VOLTS));
//            telemetry.addData("Expansion Hub used volts: ", expansionHub.getInputVoltage(VoltageUnit.VOLTS));
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
                elevators.setHorizontalDestination(Elevators.HorizontalState.HORIZONTAL_EXTENDED.state);
                horElevatorPosition = Elevators.HorizontalState.HORIZONTAL_EXTENDED.state;
            }
            flagElevatorHorizontalX = !gamepad2.cross;
//
            if(gamepad2.triangle && flagElevatorHorizontalTriangle){
                elevators.setHorizontalDestination(Elevators.HorizontalState.HORIZONTAL_RETRACTED.state);
                horElevatorPosition = Elevators.HorizontalState.HORIZONTAL_RETRACTED.state;
            }
            flagElevatorHorizontalTriangle = !gamepad2.triangle;

            if(gamepad2.square && flagElevatorHorizontalSquare){
                Actions.runBlocking(control.hangSpecimenHigh());
            }
            flagElevatorHorizontalSquare = !gamepad2.square;


            if(gamepad2.circle && flagElevatorHorizontalCircle){
                elevators.setHorizontalDestination(Elevators.HorizontalState.HORIZONTAL_HALFWAY.state);
                horElevatorPosition = Elevators.HorizontalState.HORIZONTAL_HALFWAY.state;
            }
            flagElevatorHorizontalCircle = !gamepad2.circle;
            if(isInitialized && secondery)
                elevators.updateVert();
//            colorSensorSystem.updateAlert();

            if(gamepad2.right_bumper && !isInitialized){
                elevators.setVerticalDestination(-Elevators.VerticalState.VERTICAL_HIGH.state);
                isInitialized = true;
                flagElevatorHorizontalSquare = false;
            }

            if(gamepad2.right_bumper && isInitialized && flagRightBumper){
                elevators.resetVert();
                elevators.setVerticalDestination(0);
                secondery = true;
            }
            flagRightBumper = !gamepad2.right_bumper;

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
