package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.generated.TunerConstants;

public class Constants 
{
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics
    (
        new Translation2d(TunerConstants.kFrontRightXPosInches, TunerConstants.kFrontLeftYPosInches),  // Front left module
        new Translation2d(TunerConstants.kFrontRightXPosInches, TunerConstants.kFrontRightYPosInches), // Front right module
        new Translation2d(TunerConstants.kBackLeftXPosInches, TunerConstants.kBackLeftYPosInches), // Back left module
        new Translation2d(TunerConstants.kBackRightXPosInches, TunerConstants.kBackRightYPosInches) // Back right module
    );
}
