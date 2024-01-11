package frc.lib.lib2706;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.DoubleStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * AdvantageUtil defines static functions to help format data for AdvantageScope
 * 
 * This is a copied and modified/added to version from team 686.
 */
public class AdvantageUtil {
    /** Utility class, so constructor is private. */
    private AdvantageUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Deconstruct a Pose3d into a double[] for AdvantageScope.
     * 
     * @param pose The pose to deconstruct
     * @return The array for AdvantageScope
     */
    public static double[] deconstruct(Pose3d pose)
    {
        return new double[]{
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            pose.getRotation().getQuaternion().getW(),
            pose.getRotation().getQuaternion().getX(),
            pose.getRotation().getQuaternion().getY(),
            pose.getRotation().getQuaternion().getZ()
        };
    }

    /**
     * Deconstruct a list of Pose3d into a double[] for AdvantageScope.
     * 
     * @param poses The list of poses
     * @return An array of doubles
     */
    public static double[] deconstructPose3ds(List<Pose3d> poses)
    {
        double[] r = new double[0];
        for (Pose3d pose : poses) {
            r = DoubleStream.concat(Arrays.stream(r), Arrays.stream(deconstruct(pose))).toArray();
        }
        return r;
    }

    /**
     * Deconstruct a Pose2d into a double[] for AdvantageScope.
     * 
     * @param pose The pose to deconstruct
     * @return An array of doubles
     */
    public static double[] deconstruct(Pose2d pose)
    {
        return new double[]{
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }

    /**
     * Deconstruct a list of Pose2d into a double[] for AdvantageScope.
     * 
     * @param poses The list of poses to deconstruct
     * @return An array of doubles
     */
    public static double[] deconstructPose2ds(List<Pose2d> poses)
    {
        double[] r = new double[0];
        for (Pose2d pose : poses) {
            r = DoubleStream.concat(Arrays.stream(r), Arrays.stream(deconstruct(pose))).toArray();
        }
        return r;
    }

    /**
     * Reconstruct Pose3ds from an array of doubles in AdvantageScope format.
     * 
     * @param array double[] of Pose3ds in Advantagescope format
     * @return Reconstructed Pose3ds in an ArrayList
     */
    public static ArrayList<Pose3d> reconstructPose3d(double[] array)
    {
        ArrayList<Pose3d> r = new ArrayList<Pose3d>();
        for(int i = 0; i < Math.ceil(array.length/7); i++)
        {
            r.add(new Pose3d(new Translation3d(array[0+i],array[1+i],array[2+i]), 
                             new Rotation3d(
                             new Quaternion(array[3+i],array[4+i],array[5+i],array[6+i]))));
        }
        return r;
    }

    /**
     * Reconstruct Pose2ds from an array of doubles in AdvantageScope format.
     * 
     * @param array double[] of Pose2ds in Advantagescope format
     * @return Reconstructed Pose2ds in an ArrayList
     */
    public static ArrayList<Pose2d> reconstructPose2d(double[] array)
    {
        ArrayList<Pose2d> r = new ArrayList<Pose2d>();
        for(int i = 0; i < Math.ceil(array.length/3); i++)
        {
            r.add(new Pose2d(new Translation2d(array[0+i],array[1+i]), 
                             new Rotation2d(array[2+i])));
        }
        return r;
    }

    /**
     * Deconstruct an array of SwerveModuleStates into an array of doubles for advantage scope.
     * 
     * @param state An array of SwerveModuleStates
     * @return An array of doubles in AdvantageScope format.
     */
    public static double[] deconstructSwerveModuleState(SwerveModuleState[] states) {
        double[] arr = new double[states.length * 2];

        for (int i = 0; i < states.length; i++) {
            arr[i*2] = states[i].angle.getRadians();
            arr[i*2+1] = states[i].speedMetersPerSecond;
        }

        return arr;
    }
}