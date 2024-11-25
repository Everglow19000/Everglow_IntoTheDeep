package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Config
@Autonomous(name="RightPath", group="Autonomous")
public class RightPath extends LinearOpMode {
    @Override
    public void runOpMode()  throws InterruptedException{
        Pose2d beginPose = new Pose2d(20, -63,   (1./2)*Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Claws claws  = new Claws(this);
        Elevators elevators  = new Elevators(this);

        TrajectoryActionBuilder B_sample1 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(36,-34),0.75*Math.PI - Math.PI /2);

        TrajectoryActionBuilder B_unload1 = B_sample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(52,-50),-Math.PI /2);

        TrajectoryActionBuilder B_sample2 = B_unload1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(48, -34),0.75*Math.PI- Math.PI /2);

        TrajectoryActionBuilder B_unload2 = B_sample2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(54,-50),-Math.PI /2);

        TrajectoryActionBuilder B_sample3 = B_unload2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(57, -34),0.75*Math.PI- Math.PI /2);

        TrajectoryActionBuilder B_unload3 = B_sample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(56,-50),-Math.PI /2);

        TrajectoryActionBuilder B_sample4 = B_unload3.endTrajectory().fresh()
                .setTangent(Math.PI * 0.5)
                .splineToSplineHeading(new Pose2d(26,0, Math.PI),Math.PI);

        TrajectoryActionBuilder B_unload4 = B_sample4.endTrajectory().fresh()
                .setTangent(-(0.75)*Math.PI + Math.PI / 2)
                .splineToSplineHeading(new Pose2d(58,-50,- Math.PI / 2),- Math.PI / 2);

        // All the actions are Trajectory Builders, need to be converted to actions
        Action sample1 = B_sample1.build();
        Action unload1 = B_unload1.build();

        Action sample2 = B_sample2.build();
        Action unload2 = B_unload2.build();

        Action sample3 = B_sample3.build();
        Action unload3 = B_unload3.build();

        Action sample4 = B_sample4.build();
        Action unload4 = B_unload4.build();

//        Actions.runBlocking(v_e to 0 and h_e to 0)
//        Actions.runBlocking(set elevator power to 0.8)

        Actions.runBlocking(
                new SequentialAction(
                        sample1,
                        //pick up sample
                        unload1,
                        //release sample
                        sample2,
                        //pick up sample
                        unload2,
                        //release sample
                        sample3,
                        //pick up sample
                        unload3,
                        //release sample
                        sample4,
                        //pick up sample
                        unload4

                )
        );
    }
}