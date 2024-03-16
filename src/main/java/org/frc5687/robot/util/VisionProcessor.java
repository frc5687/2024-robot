package org.frc5687.robot.util;

import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.frc5687.Messages.VisionPoseArray;
import org.frc5687.Messages.VisionPose;

public class VisionProcessor {
    private NetworkTable _visionTable;
    private RawSubscriber _visionPosesRawEntry;

    private DetectedNoteArray _detectedObjects = new DetectedNoteArray();

    public VisionProcessor() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        _visionTable = inst.getTable("VisionProcessor");
        _visionPosesRawEntry = _visionTable.getRawTopic("vision_poses_raw").subscribe("raw", new byte[0], PubSubOption.periodic(0.01),PubSubOption.sendAll(true));
    }

    public DetectedNoteArray getDetectedObjects() {
        try {
            byte[] rawData = _visionPosesRawEntry.get(new byte[0]);
            ByteBuffer bb = ByteBuffer.wrap(rawData);
            bb.order(ByteOrder.LITTLE_ENDIAN);

            VisionPoseArray visionPoseArray = VisionPoseArray.getRootAsVisionPoseArray(bb);
            int numObjects = visionPoseArray.posesLength();

            DetectedNote[] notes = new DetectedNote[numObjects];
            for (int i = 0; i < numObjects; i++) {
                VisionPose visionPose = visionPoseArray.poses(i);
                int id = visionPose.id();
                double x = visionPose.x();
                double y = visionPose.y();
                double z = visionPose.z();
                long detectionTimeMs = visionPose.timestamp();

                Translation3d translation = new Translation3d(x, y, z);
                Rotation3d rotation = new Rotation3d();
                Pose3d pose = new Pose3d(translation, rotation);

                notes[i] = new DetectedNote(id, pose, detectionTimeMs);
            }

            _detectedObjects = new DetectedNoteArray(notes);
            return _detectedObjects;
        } catch (Exception e) {
            // DriverStation.reportError("RobotState.getDetectedObjects failed due to exception "+e.getMessage(), false);
            return new DetectedNoteArray(new DetectedNote[0]);
        }
    }

    public static class DetectedNote {
        private final int _id;
        private final Pose3d _pose;
        private final long _detectionTimeMs;

        public DetectedNote(int id, Pose3d pose, long detectionTimeMs) {
            _id = id;
            _pose = pose;
            _detectionTimeMs = detectionTimeMs;
        }

        public int getId() {
            return _id;
        }

        public Pose3d getPose() {
            return _pose;
        }

        public long getDetectionTimeMs() {
            return _detectionTimeMs;
        }
    }

    public static class DetectedNoteArray {
        private final DetectedNote[] _notes;

        public DetectedNoteArray(DetectedNote[] notes) {
            _notes = notes;
        }

        public DetectedNoteArray() {
            _notes = new DetectedNote[0];
        }

        public DetectedNote[] getNotes() {
            return _notes;
        }
    }
}