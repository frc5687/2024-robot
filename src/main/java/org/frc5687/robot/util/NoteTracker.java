package org.frc5687.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public class NoteTracker {
    private List<TrackedNote> _trackedNotes;
    private final double proximityThreshold = 0.1; // distance between measurements in meters

    public NoteTracker() {
        _trackedNotes = new ArrayList<>();
    }

    public void updateTrackedNotes(List<Pose2d> detectedNotes) {
        double currentTimeMs = Timer.getFPGATimestamp();

        // Update or add new notes based on proximity
        for (Pose2d detectedPose : detectedNotes) {
            boolean noteUpdated = false;
            for (TrackedNote trackedNote : _trackedNotes) {
                if (trackedNote.getPose().getTranslation()
                        .getDistance(detectedPose.getTranslation()) <= proximityThreshold) {
                    trackedNote.setPose(detectedPose);
                    trackedNote.setLastSeenMs(currentTimeMs);
                    noteUpdated = true;
                    break;
                }
            }
            if (!noteUpdated) {
                _trackedNotes.add(new TrackedNote(detectedPose, currentTimeMs));
            }
        }

        // Remove notes not seen recently
        double visibilityTimeoutMs = 0.25; 
        _trackedNotes.removeIf(note -> currentTimeMs - note.getLastSeenMs() > visibilityTimeoutMs);
    }

    public List<Pose2d> getTrackedNotes() {
        List<Pose2d> poses = new ArrayList<>();
        for (TrackedNote note : _trackedNotes) {
            poses.add(note.getPose());
        }
        return poses;
    }

    // These are notes that are field relative not camera
    private static class TrackedNote {
        private Pose2d _pose;
        private double _lastSeenMs;

        public TrackedNote(Pose2d pose, double lastSeenMs) {
            _pose = pose;
            _lastSeenMs = lastSeenMs;
        }

        public Pose2d getPose() {
            return _pose;
        }

        public void setPose(Pose2d pose) {
            _pose = pose;
        }

        public double getLastSeenMs() {
            return _lastSeenMs;
        }

        public void setLastSeenMs(double lastSeenMs) {
            _lastSeenMs = lastSeenMs;
        }
    }
}
