package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem 
{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    // Pose Estimator to fuse odometry and vision data
    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d poseMeters = new Pose2d();

    private Pigeon2 pigeon = new Pigeon2(TunerConstants.kPigeonId);


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,SwerveModuleConstants... modules) 
        {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }

        // Initialize the Pose Estimator
        poseEstimator = new SwerveDrivePoseEstimator
        (
                Constants.KINEMATICS,
                getHeading(),
                getModulePositions(),
                new Pose2d(),
                stateStdDevs(),
                visionMeasurementStdDevs());
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) 
    {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) 
        {
            startSimThread();
        }

        // Initialize the Pose Estimator
        poseEstimator = new SwerveDrivePoseEstimator
        (
                Constants.KINEMATICS,
                getHeading(),
                getModulePositions(),
                new Pose2d(),
                stateStdDevs(),
                visionMeasurementStdDevs());
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) 
    {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() 
    {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> 
        {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() 
    {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) 
        {
            DriverStation.getAlliance().ifPresent((allianceColor) -> 
            {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        // Update odometry with fused vision measurements
        updateOdometry();
    }

    /**
     * Updates the robot's odometry and incorporates vision measurements from Limelights.
     */
    public void updateOdometry() 
    {
        // Update the pose estimator with the latest module states and gyro heading
        poseEstimator.update(
                getHeading(),
                getModulePositions());

        double currentTime = Timer.getFPGATimestamp();

        // Front Limelight
        Pose2d visionPoseFront = getVisionPose(RobotContainer.limelightFront);
        if (visionPoseFront != null) 
        {
            double latencyFront = RobotContainer.limelightFront.getLatency() / 1000.0;
            poseEstimator.addVisionMeasurement(visionPoseFront, currentTime - latencyFront);
        }

        // Back Limelight
        Pose2d visionPoseBack = getVisionPose(RobotContainer.limelightBack);
        if (visionPoseBack != null) 
        {
            double latencyBack = RobotContainer.limelightBack.getLatency() / 1000.0;
            poseEstimator.addVisionMeasurement(visionPoseBack, currentTime - latencyBack);
        }

        // Update the stored pose
        poseMeters = poseEstimator.getEstimatedPosition();
    }

    /**
     * Retrieves the robot's pose from the Limelight's onboard calculations.
     *
     * @param limelight The Limelight to get data from.
     * @return The estimated Pose2d from vision data, or null if no target is found.
     */
    public Pose2d getVisionPose(Limelight limelight) 
    {
        if (!limelight.hasTarget()) 
        {
            return null;
        }

        double[] botpose = limelight.getBotPose();
        if (botpose.length < 6) 
        {
            // Incomplete data received
            return null;
        }

        // Extract pose data from botpose array
        double x = botpose[0]; // X position in meters
        double y = botpose[1]; // Y position in meters
        double thetaDegrees = botpose[5]; // Yaw in degrees
        Rotation2d rotation = Rotation2d.fromDegrees(thetaDegrees);

        return new Pose2d(x, y, rotation);
    }

    /**
     * Returns the robot's estimated pose.
     *
     * @return The current Pose2d.
     */
    public Pose2d getPose() 
    {
        return poseMeters;
    }

    /**
     * Returns the robot's heading as a Rotation2d.
     *
     * @return The current heading.
     */
    public Rotation2d getHeading() 
    {
        return Rotation2d.fromDegrees(getPigeon().getYaw().getValue());
    }

    /**
     * Returns the positions of the swerve modules.
     *
     * @return An array of SwerveModulePosition.
     */
    public SwerveModulePosition[] getModulePositions() 
    {
        return new SwerveModulePosition[] 
        {
                this.getModule(0).getPosition(true),
                this.getModule(1).getPosition(true),
                this.getModule(2).getPosition(true),
                this.getModule(3).getPosition(true)
        };
    }

    public Pigeon2 getPigeon() 
    {
        return pigeon;
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Defines the standard deviations for the robot's state estimation.
     *
     * @return A matrix of standard deviations.
     */
    private static Matrix<N3, N1> stateStdDevs() 
    {
        // x, y, theta (radians)
        return VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    }

    /**
     * Defines the standard deviations for the vision measurements.
     *
     * @return A matrix of standard deviations.
     */
    private static Matrix<N3, N1> visionMeasurementStdDevs() 
    {
        // x, y, theta (radians)
        return VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
    }
}
