package frc.robot.util;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

public class PathUtils {

    /**
     * 
     */
    public static final PathConstraints pathConstraints = new PathConstraints(
        SWERVE.MAX_SPEED_METERS_PER_SECOND * 0.85,
        SWERVE.MAX_ACCELERATION * 0.50,
        SWERVE.MAX_ROTATIONAL_SPEED,
        SWERVE.MAX_ANGULAR_SPEED
    );

    public static final PathConstraints fastPathConstraints = new PathConstraints(
        SWERVE.MAX_SPEED_METERS_PER_SECOND,
        SWERVE.MAX_ACCELERATION * 0.7,
        SWERVE.MAX_ROTATIONAL_SPEED,
        SWERVE.MAX_ANGULAR_SPEED
    );

    public static Command getPathToPose(Pose2d desiredPose, Supplier<Double> goalEndVel) {
        return AutoBuilder.pathfindToPose(desiredPose, fastPathConstraints, goalEndVel.get());
    }

    /**
     * Generate a PathPlannerPath from a supplied list of poses.
     * <P>
     * Convenience method to generate a PathPlannerPath from a list of poses. The
     * rotation of the last pose us used for the end state rotation of the path.
     * The required path constraints are taken from the swerve drive constants.
     * @param poses The poses of the waypoints for the path.
     * @return The constructed path.
     */
    // public static PathPlannerPath generatePath(Pose2d... poses) {
    //     return PathUtils.generatePath(poses[poses.length-1].getRotation(), poses);
    // }

    /**
     * Generate a PathPlannerPath from a supplied list of poses.
     * <P>
     * Convenience method to generate a PathPlannerPath from a provided end state
     * rotation and a list of poses. The required path constraints are taken from
     * the swerve drive constants.
     * @param endStateRotation The desired end state rotation (not heading).
     * @param poses The poses of the waypoints for the path.
     * @return The constructed path.
     */
    // public static PathPlannerPath generatePath(Rotation2d endStateRotation, Pose2d... poses) {   
    //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);
    //     return new PathPlannerPath(
    //         bezierPoints,
    //         PathUtils.pathConstraints,
    //         new GoalEndState(0.0, endStateRotation, true));
    // }

    // public static PathPlannerPath generatePath(PathConstraints constraints, Pose2d... poses) {
    //     return PathUtils.generatePath(poses[poses.length-1].getRotation(), constraints, poses);
    // }

    // public static PathPlannerPath generatePath(Rotation2d endStateRotation, PathConstraints constraints, Pose2d... poses) {   
    //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);
    //     return new PathPlannerPath(
    //         bezierPoints,
    //         constraints,
    //         new GoalEndState(0.0, endStateRotation, true));
    // }

    /**
     * Generate a WPILib trajectory from a PathPlannerPath for display on a Field2d widget.
     * <P>
     * NOTE: This should only be used for displaying a PathPlannerPath as a Trajectory on
     * a Field2d widget. The generated trajectory is not meant to run on the robot.
     * @param path The PathPlannerPath to generate the Trajectory from.
     * @return The generated Trajectory.
     */
    public static Trajectory TrajectoryFromPath(PathPlannerPath path, RobotConfig config) {
        PathPlannerTrajectory traj = path.generateTrajectory(new ChassisSpeeds(0.0,0.0,0.0), path.getInitialHeading(), config);
        List<Trajectory.State> states = new ArrayList<Trajectory.State>();
        for (PathPlannerTrajectoryState state : traj.getStates()) {
            states.add(
              new State(
                state.timeSeconds,
                state.linearVelocity,
                0.0,
                state.pose,
                0.0
              )
            );
        }
        return new Trajectory(states);
    }


}
