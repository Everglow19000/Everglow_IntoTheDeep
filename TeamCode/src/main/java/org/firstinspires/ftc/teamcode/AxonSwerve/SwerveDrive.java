package org.firstinspires.ftc.teamcode.AxonSwerve;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.trajectory.constraint.SwerveDriveKinematicsConstraint;

import java.util.List;

public class SwerveDrive {
    SwerveModule[] modules = new SwerveModule[4];
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];

    public Pose2d pose;


//    public final class FollowTrajectoryAction implements Action {
//        public final TimeTrajectory trajectory;
//        private double beginTimeSeconds = -1;
//
//        public FollowTrajectoryAction(TimeTrajectory t) {
//            trajectory = t;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            double t;
//            if (beginTimeSeconds < 0) {
//                beginTimeSeconds = Actions.now();
//                t = 0;
//            }
//            else {
//                t = Actions.now() - beginTimeSeconds;
//            }
//
//            if (t > trajectory.duration) {
//                modules[0].setDrivePower(0);
//                modules[1].setDrivePower(0);
//                modules[2].setDrivePower(0);
//                modules[3].setDrivePower(0);
//
//                return false;
//            }
//
//            Pose2dDual<Time> txWorldTarget = trajectory.get(t);
//
//            PoseVelocity2d robotVelocityRobot = updatePoseEstimate();
//        }
//    }
//
//    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
//        return new TrajectoryActionBuilder(
//                TurnAction::new,
//                FollowTrajectoryAction::new,
//                new TrajectoryBuilderParams(
//                        1e-6,
//                        new ProfileParams(
//                                0.25, 0.1, 1e-2
//                        )
//                ),
//                beginPose, 0.0,
//                defaultTurnConstraints,
//                defaultVelConstraint, defaultAccelConstraint
//        );
//    }
}
