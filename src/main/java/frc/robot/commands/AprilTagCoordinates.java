package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class AprilTag {
    double x, y, z;
    double zRotation, yRotation;
    
    public AprilTag(double x, double y, double z, double zRotation, double yRotation) {
        this.zRotation = zRotation - 180;
        this.yRotation = yRotation;
        this.x = ((x + (22 * Math.cos(zRotation*(3.141592653589793238462/180))))/ 39.37); // Convert inches to meters
        this.y = ((y + (22 * Math.sin(zRotation*(3.141592653589793238462/180))))/ 39.37);
        this.z = z / 39.37;
    }

    @Override
    public String toString() {
        return String.format("X: %.2f, Y: %.2f, Z: %.2f, Z-Rotation: %.1f°, Y-Rotation: %.1f°", 
                              x, y, z, zRotation, yRotation);
    }
}

public class AprilTagCoordinates {
    private static final Map<Integer, AprilTag> tagMap = new HashMap<>();

    static {
        tagMap.put(1, new AprilTag(467.64, 292.31, 35.00, 180, 0));
        tagMap.put(2, new AprilTag(469.11, 182.6, 44.25, 90, 0));
        tagMap.put(3, new AprilTag(445.35, 172.84, 44.25, 180, 0));
        tagMap.put(4, new AprilTag(445.35, 158.84, 44.25, 180, 0));
        tagMap.put(5, new AprilTag(469.11, 135.09, 44.25, 270, 30));
        tagMap.put(6, new AprilTag(467.64, 25.37, 35.00, 180, 0));
        tagMap.put(7, new AprilTag(470.59, 25.37, 35.00, 0, 0));
        tagMap.put(8, new AprilTag(483.11, 135.09, 44.25, 270, 0));
        tagMap.put(9, new AprilTag(492.88, 144.84, 44.25, 0, 0));
        tagMap.put(10, new AprilTag(492.88, 158.84, 44.25, 0, 0));
        tagMap.put(11, new AprilTag(483.11, 182.60, 44.25, 90, 0));
        tagMap.put(12, new AprilTag(470.59, 292.31, 35.00, 0, 0));
        tagMap.put(13, new AprilTag(650.92, 291.47, 21.75, 180, 0));
        tagMap.put(14, new AprilTag(650.92, 274.47, 21.75, 180, 0));
        tagMap.put(15, new AprilTag(650.90, 170.22, 21.75, 180, 0));
        tagMap.put(16, new AprilTag(650.90, 153.22, 21.75, 180, 0));
        tagMap.put(17, new AprilTag(183.59, 25.37, 35.00, 0, 0));
        tagMap.put(18, new AprilTag(182.11, 135.09, 44.25, 270, 0));
        tagMap.put(19, new AprilTag(205.87, 144.84, 44.25, 0, 0));
        tagMap.put(20, new AprilTag(205.87, 158.84, 44.25, 0, 0));
        tagMap.put(21, new AprilTag(182.11, 182.60, 44.25, 90, 0));
        tagMap.put(22, new AprilTag(183.59, 292.31, 35.00, 0, 0));
        tagMap.put(23, new AprilTag(180.64, 292.31, 35.00, 180, 0));
        tagMap.put(24, new AprilTag(168.11, 182.60, 44.25, 90, 0));
        tagMap.put(25, new AprilTag(158.34, 172.84, 44.25, 180, 0));
        tagMap.put(26, new AprilTag(158.34, 158.84, 44.25, 180, 0));
        tagMap.put(27, new AprilTag(168.11, 135.09, 44.25, 180, 0));
        tagMap.put(28, new AprilTag(180.64, 25.37, 35.00, 180, 0));
        tagMap.put(29, new AprilTag(0.30, 26.22, 21.75, 0, 0));
        tagMap.put(30, new AprilTag(0.30, 43.22, 21.75, 0, 0));
        tagMap.put(31, new AprilTag(0.32, 147.47, 21.75, 0, 0));
        tagMap.put(32, new AprilTag(0.32, 164.47, 21.75, 0, 0));
    }

    public static AprilTag getAprilTag(int id) {
        return tagMap.get(id);
    }

    public static Double getX(int tagId) {
        AprilTag tag = tagMap.get(tagId);
        return (tag != null) ? tag.x : null;
    }
    
    public static Double getY(int id) {
        AprilTag tag = tagMap.get(id);
        return (tag != null) ? tag.y : null;
    }
    
    public static Double getZ(int id) {
        AprilTag tag = tagMap.get(id);
        return (tag != null) ? tag.z : null;
    }

    public static Pose2d getPose2d(int tagId) {
    AprilTag tag = tagMap.get(tagId);
    if (tag != null) {
        return new Pose2d(tag.x, tag.y, Rotation2d.fromDegrees(tag.zRotation));
    }
    return null;
}
    public static void main(String[] args) {
        int testId = 1; // Change this to test different IDs
        AprilTag tag = getAprilTag(testId);
        if (tag != null) {
            System.out.println("AprilTag " + testId + ": " + tag);
        } else {
            System.out.println("AprilTag ID not found.");
        }
    }
}
