package frc.robot.util;

import java.util.Set;

import org.parts3492.partslib.command.PARTsCommandUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;

public class Trench {
    private static Pose2d goal;

    public static Command parkUnderTrench(PARTsDrivetrain drivetrain) {
        return PARTsCommandUtils.setCommandName("Trench.parkUnderTrench", new DeferredCommand(
                () -> drivetrain
                        .commandPathFindToPose(getNearestTrench(drivetrain.getPose(), Field.getAllianceTrenchPoses())),
                Set.of(drivetrain)));
        /*return PARTsCommandUtils.setCommandName("Trench.parkUnderTrench",
                Commands.runOnce(() -> goal = getNearestTrench(drivetrain.getPose(), Field.getAllianceTrenchPoses()))
                        .andThen(drivetrain.commandPathFindToPose(goal)));*/
    }

    private static Pose2d getNearestTrench(Pose2d current, Pose2d[] poses) {
        int index = 0;
        double distance = getDistance(current, poses[0]);
        for (int i = 0; i < poses.length; i++) {
            double localDistance = getDistance(current, poses[i]);
            if (localDistance < distance) {
                distance = localDistance;
                index = i;
            }
        }
        return poses[index];
    }

    private static double getDistance(Pose2d current, Pose2d goal) {
        return Math.sqrt(Math.pow(current.getX() - goal.getX(), 2) + Math.pow(current.getY() - goal.getY(), 2));
    }
}
