// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706;

import java.util.NoSuchElementException;
import java.util.Optional;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

/** 
 * PoseBuffer stores a buffer of the last 1.5 seconds of poses given to it.
 * Intended to store the last 1.5 seconds of Odometry poses for vision latency compensation.
 * 
 * The concept of this class is pulled out of PoseEstimator classes by WPILib.
 */
public class PoseBuffer {
    private final double BUFFER_DURATION = 1.5;

    private final TimeInterpolatableBuffer<Pose2d> m_poseBuffer = TimeInterpolatableBuffer
            .createBuffer(BUFFER_DURATION);

    /**
     * Add a pose to the buffer.
     * Automatically assigns the current time as the timestamp.
     * 
     * @param pose The new Pose2d to add.
     */
    public void addPoseToBuffer(Pose2d pose) {
        m_poseBuffer.addSample(
            MathSharedStore.getTimestamp(),
            pose);
    }

    /**
     * Get a pose at the given timestamp. 
     * Returns an empty Optional if the buffer is empty or doesn't go back far enough.
     * 
     * @param timestampSeconds The timestamp for the pose to get, matching WPILib PoseEstimator's 
     *                         timestamps (which matches PhotonVision and Limelight)
     * @return An Optional of the Pose2d or an empty Optional.
     */
    public Optional<Pose2d> getPoseAtTimestamp(double timestampSeconds) {
        // If this measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (m_poseBuffer.getInternalBuffer().lastKey() - BUFFER_DURATION > timestampSeconds) {
                return Optional.empty();
            }
        } catch (NoSuchElementException ex) {
            return Optional.empty();
        }

        // Get the pose odometry measured at the moment the vision measurement was made.
        return m_poseBuffer.getSample(timestampSeconds);
    }

    /**
     * Clear the buffer. 
     * For use with odometry, must be called when odometry is reset.
     */
    public void clearBuffer() {
        m_poseBuffer.clear();
    }
}