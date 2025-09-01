package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OpMode.ActionSequantionalOpMode.linearToExpo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.Sweeper;
import org.firstinspires.ftc.teamcode.Systems.Token.Token;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenAction;
import org.firstinspires.ftc.teamcode.Systems.Token.Tokenable;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class MecanumDrive{
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        // drive model parameters
        public double inPerTick = 0.00304934967951;
        public double lateralInPerTick = 0.0023032186757462607;
        public double trackWidthTicks = 4826.661865469477;

        // feedforward parameters (in tick units)
        public double kS = 0.9368459905611335;
        public double kV = 0.000536546652326513;
        public double kA = 0.00003;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 10;
        public double lateralGain = 10;
        public double headingGain = 1; // shared with turn

        public double axialVelGain = 1;
        public double lateralVelGain = 1;
        public double headingVelGain = 1; // shared with turn
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx frontLeft, backLeft, backRight, frontRight;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    public Pose2d pose;

    public final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private double lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;

        public DriveLocalizer() {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.frontLeft));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.backLeft));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.backRight));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.frontRight));

            imu = lazyImu.get();

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        frontLeft.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        backLeft.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        backRight.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        frontRight.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            // list of time values
            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));

            // create the desired x and y points based on the list of time values
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            // record start time and current time
            double t; // current time
            if (beginTs < 0) {
                beginTs = Actions.now(); // start time
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            // stop if going on for too long
            if (t >= timeTrajectory.duration) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);

                return false;
            }

            // check where we want to go to
            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            // dont think this is needed
//            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            // current robot velocity
            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            // calculate the wanted velocity
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            // dont think this is needed
//            driveCommandWriter.write(new DriveCommandMessage(command));

            // the wanted wheel velocities
            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            // determine power of each motor
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

            // dont think this is needed
//            mecanumCommandWriter.write(new MecanumCommandMessage(
//                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
//            ));

            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);
            frontRight.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            frontLeft.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            backLeft.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            backRight.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            frontRight.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public static double linearInputToExponential(double power){
        double base = 6;
        return (Math.pow(base, Math.abs(power)) - 1) / (base - 1) * Math.signum(power);
    }

    public class MecanumDriveAction extends TokenAction{
        private GamepadEx gamepad1, gamepad2;
        private  Sweeper sweeper;
        Token token;
        public MecanumDriveAction(GamepadEx gamepad1, GamepadEx gamepad2, Sweeper sweeper, Tokenable Token, Token stopToken){
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
            isDone = Token;
            isInitialized = true;
            this.sweeper = sweeper;
            token = stopToken;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gamepad1.readButtons();
            gamepad2.readButtons();

            setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            linearToExpo(gamepad1.getLeftY())*(1.0/Math.pow(4.5, gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))),
                            -gamepad1.getLeftX()*(1.0/Math.pow(4, gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
                    ),
                    -gamepad1.getRightX()*(1.0/Math.pow(5, gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
            ));
            updatePoseEstimate();


            if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                sweeper.setPosition(Sweeper.SweeperAngle.SWEEPER_EXTENDED);
            }
            else {
                sweeper.setPosition(Sweeper.SweeperAngle.SWEEPER_RETRACTED);
            }

            if(gamepad2.wasJustPressed(GamepadKeys.Button.B))
                token.Interrupt();
            return !checkToken();
        }
    }

    public class DriveUntilStop extends TokenAction{
        ColorSensorSystem colorSensorSystem;

        Telemetry telemetry;


        //TODO: change the following powers
        double power = 0.8;

        Token stopToken;

        public double angle = 0;


        boolean recLeft;
        boolean recRight;

        boolean leftDetected = false;
        boolean rightDetected = false;

        int turnMultiplier = 0;

        boolean finishedCalculatingRotation = false;

        boolean startedRotating = false;

        double timeOfStartMillis = -1;

        private DriveUntilStop(ColorSensorSystem colorSensorSystem, Token stopToken, Telemetry telemetry){
            this.colorSensorSystem = colorSensorSystem;
            this.stopToken = stopToken;
            this.telemetry = telemetry;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timeOfStartMillis == -1) {
                timeOfStartMillis = System.currentTimeMillis();
            }
            power = System.currentTimeMillis() - timeOfStartMillis < 500 ? 0.8 : 0.3;
            recLeft = colorSensorSystem.isOnTape(false);
            recRight = colorSensorSystem.isOnTape(true);

            if (recLeft && !leftDetected) {
                leftDetected = true;
                if (!rightDetected) {
                    angle = pose.heading.toDouble();
                }
            }
            if (recRight && !rightDetected) {
                rightDetected = true;
                if (!leftDetected) {
                    angle = pose.heading.toDouble();
                }
            }

            if (finishedCalculatingRotation) {
                if (!startedRotating) {
                    startedRotating = true;
                    if (!(recLeft && recRight)) { //do last correction
                        Action lastAction = actionBuilder(pose).fresh().turn((-turnMultiplier * angle)/4.0).build();
                        boolean lastRunValue;
                        do {
                            lastRunValue = lastAction.run(telemetryPacket);
                        }
                        while (lastRunValue);
                    }
                    return false;
                }
            }
            else {
                if (!leftDetected && !rightDetected) {
                    setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    power,
                                    0
                            ),
                            0
                    ));
                }
                else if (rightDetected) {
                    if (recLeft) {
                        turnMultiplier =  1;
                        angle = pose.heading.toDouble() - angle;
                        angle /= 2.0;
                        finishedCalculatingRotation = true;
                        setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        0,
                                        0
                                ),
                                0
                        ));
                    } else {
                        setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        0,
                                        0
                                ),
                                -0.2
                        ));
                    }
                }
                else if (leftDetected) {
                    if (recRight) {
                        turnMultiplier = -1;
                        angle = pose.heading.toDouble() - angle;
                        angle /= 2.0;
                        finishedCalculatingRotation = true;
                        setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        0,
                                        0
                                ),
                                0
                        ));
                    } else {
                        setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        0,
                                        0
                                ),
                                0.2
                        ));
                    }
                }
                updatePoseEstimate();
            }
            if (telemetry != null) {
                telemetry.addData("recRight", recRight);
                telemetry.addData("recLeft", recLeft);
                telemetry.update();
            }
            return true;
        }
    }

    public MecanumDriveAction getMecanumDriveAction(GamepadEx gamepad1, GamepadEx gamepad2, Sweeper sweeper
            , Tokenable tokenable, Token stopToken){
        return new MecanumDriveAction(gamepad1,gamepad2, sweeper, tokenable, stopToken);
    }

    public DriveUntilStop getDriveUntilStopAction(ColorSensorSystem colorSensorSystem, Token stopToken){
        return new DriveUntilStop(colorSensorSystem, stopToken, null);
    }

    public DriveUntilStop getDriveUntilStopAction(ColorSensorSystem colorSensorSystem, Token stopToken, Telemetry telemetry){
        return new DriveUntilStop(colorSensorSystem, stopToken, telemetry);
    }
}
