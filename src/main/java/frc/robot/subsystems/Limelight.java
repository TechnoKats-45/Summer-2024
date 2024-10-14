package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable table;

    public Limelight(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double getLatency() {
        return table.getEntry("tl").getDouble(0) + 11; // 11 ms image capture latency
    }

    /**
     * Retrieves the robot's pose as estimated by the Limelight.
     *
     * @return An array containing [x, y, z, pitch, yaw, roll].
     */
    public double[] getBotPose() {
        return table.getEntry("botpose").getDoubleArray(new double[6]);
    }
}
