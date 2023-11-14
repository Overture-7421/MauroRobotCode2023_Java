package frc.lib.subsystems;

//import com.ctre.phoenix6.Timestamp;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics. SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

//import frc.lib.subsystems.SwerveModule;

public class SwerveChassis extends SubsystemBase {

    private AHRS navx = new AHRS();

    private SwerveModule m_frontLeftModule;
    private SwerveModule m_frontRightModule;
    private SwerveModule m_backLeftModule;
    private SwerveModule m_backRightModule;

    private double linearX = 0;
    private double linearY = 0;
    private double angular = 0;

    SwerveDriveKinematics kinematics;
    SwerveModulePosition[] odometryPos;
    SwerveDrivePoseEstimator odometry;

    /**
     * Creates a new SwerveChassis.
     */
    public SwerveChassis() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e){
            e.printStackTrace();
        }

        double startTime = Timer.getFPGATimestamp();
        while (navx.isCalibrating()){
            double timePassed = Timer.getFPGATimestamp() - startTime;
            if (timePassed > 10){
                System.out.println("NavX calibration timed out");
                break;
            }
            try{
                Thread.sleep(10);
            } catch (InterruptedException e){
                e.printStackTrace();
            }
        }

        navx.zeroYaw();
        
    }
    
    /**
     * Sets the swerve module positions for the kinematics and odometry.
     * 
     * @param positions
     */
    public void setModulePositions(Translation2d[] positions) {
        kinematics = new SwerveDriveKinematics(positions);
    }

    /**
     * Sets the swerve modules ratios
     * 
     * @param turnRatio
     * @param driveRatio
     * @param wheelDiameter
     */
    public void setModulesRatios(double turnRatio, double driveRatio, double wheelDiameter){
        m_frontRightModule.setGearRatio(turnRatio, driveRatio);
        m_frontLeftModule.setGearRatio(turnRatio, driveRatio);
        m_backRightModule.setGearRatio(turnRatio, driveRatio);
        m_backLeftModule.setGearRatio(turnRatio, driveRatio);

        m_frontRightModule.setWheelDiameter(wheelDiameter);
        m_frontLeftModule.setWheelDiameter(wheelDiameter);
        m_backRightModule.setWheelDiameter(wheelDiameter);
        m_backLeftModule.setWheelDiameter(wheelDiameter);
    };

    public void setModules(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight){
        m_frontLeftModule = frontLeft;
        m_frontRightModule = frontRight;
        m_backLeftModule = backLeft;
        m_backRightModule = backRight;
        
        odometryPos = new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(),
            m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(),
            m_backRightModule.getPosition()
        };

        odometry = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(0),
            odometryPos,
            new Pose2d(0, 0, new Rotation2d(0)));
    }

    /**
     * Sets each module rotator PID values
     */
    public void setRotatorPID(double kP, double kI, double kD){
        m_backRightModule.setRotatorPIDValues(kP, kI, kD);
        m_backLeftModule.setRotatorPIDValues(kP, kI, kD);
        m_frontRightModule.setRotatorPIDValues(kP, kI, kD);
        m_frontLeftModule.setRotatorPIDValues(kP, kI, kD);
    }

    /**
     * Sets each module drive PID values
     */
    public void setDrivePID(double kP, double kI, double kD){
        m_backRightModule.setDrivePIDValues(kP, kI, kD);
        m_backLeftModule.setDrivePIDValues(kP, kI, kD);
        m_frontRightModule.setDrivePIDValues(kP, kI, kD);
        m_frontLeftModule.setDrivePIDValues(kP, kI, kD);
    }

    /**
     * Sets the modules feedforward values
     */
    public void setFeedForward(double kS, double kV, double kA){
        m_backRightModule.setFFConstants(kS, kV, kA);
        m_backLeftModule.setFFConstants(kS, kV, kA);
        m_frontRightModule.setFFConstants(kS, kV, kA);
        m_frontLeftModule.setFFConstants(kS, kV, kA);
    }

    /**
     * Sets the robot target speed
     * 
     * @param speeds ChassisSpeeds object
     */
    public void setSpeed(ChassisSpeeds speeds){
        linearX = speeds.vxMetersPerSecond;
        linearY = speeds.vyMetersPerSecond;
        angular = speeds.omegaRadiansPerSecond;

        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);

        setModuleStates(desiredStates);
    }

    /**
     * Returns the robot odometry
     * 
     * @return Pose2d object
     */
    public Pose2d getOdometry() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Resets the robot odometry
     * 
     * @param initPose Pose2d object
     */
    public void resetOdometry(Pose2d initPose) {
        odometry.resetPosition(new Rotation2d(-navx.getAngle()), getModulePosition(), initPose);
    }

    /**
     * Return the robot kinematics
     * 
     * @return SwerveDriveKinematics object
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Updates odometry using vision
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp){
        odometry.addVisionMeasurement(pose, timestamp);
    }

    /**
     * Sets the navx to desired angle
     * 
     * @param angle Desired angle
     */
    public void resetNavx(double angle) {
        Pose2d actualOdometry = getOdometry();
        Pose2d newOdometry = new Pose2d(actualOdometry.getX(), actualOdometry.getY(), Rotation2d.fromDegrees(angle));
        resetOdometry(newOdometry);
    }

    /**
     * Updates module states
     * 
     * @param desiredStates SwerveModuleStates array
     */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        m_frontLeftModule.setState(desiredStates[0]);
        m_frontRightModule.setState(desiredStates[1]);
        m_backLeftModule.setState(desiredStates[2]);
        m_backRightModule.setState(desiredStates[3]);

        m_backRightModule.setVoltages();
        m_backLeftModule.setVoltages();
        m_frontRightModule.setVoltages();
        m_frontLeftModule.setVoltages();
    }

    /**
     * Returns the module states
     * 
     * @return SwerveModuleState array
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeftModule.getState(),
            m_frontRightModule.getState(),
            m_backLeftModule.getState(),
            m_backRightModule.getState()
        };
    }

    /**
     * Returns the module positions
     * 
     * @return SwerveModulePosition array
     */
    public SwerveModulePosition[] getModulePosition(){
        return new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(),
            m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(),
            m_backRightModule.getPosition()
        };
    }

    /**
     * Returns the robot pitch
     * 
     * @return double
     */
    public double getPitch() {
        return navx.getPitch();
    }

    /**
     * Returns the robot yaw
     * 
     * @return double
     */
    public double getYaw() {
        return getOdometry().getRotation().getDegrees();
    }

    /**
     * Returns the robot roll
     * 
     * @return double
     */
    public double getRoll() {
        return navx.getRoll();
    }

    /**
     * Updates the robot odometry
     */
    public void updateOdometry() {
        odometry.update(Rotation2d.fromDegrees(-navx.getAngle()), getModulePosition());
    }

    public void shuffleboardPeriodic() {
       SmartDashboard.putNumber("LinearX", linearX);
       SmartDashboard.putNumber("LinearY", linearY);
       SmartDashboard.putNumber("Angular", angular);

        Pose2d estimatedPos = getOdometry();
        SmartDashboard.putNumber("Roll", getRoll());

        SmartDashboard.putNumber("OdometryX", estimatedPos.getX());
        SmartDashboard.putNumber("OdometryY", estimatedPos.getY());
        SmartDashboard.putNumber("AnglenaveX", estimatedPos.getRotation().getDegrees());
    }
}
