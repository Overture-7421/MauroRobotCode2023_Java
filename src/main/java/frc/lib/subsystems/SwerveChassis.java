package frc.lib.subsystems;

import com.ctre.phoenix6.Timestamp;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics. SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import frc.lib.subsystems.SwerveModule;

public class SwerveChassis extends SubsystemBase {
    // navx pendinte

    private double linearX;
    private double linearY;
    private double angular;

    private AHRS navAhrs;

    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private double wheelVoltage;
    private double targetAngle;

    private Translation2d modulePos; // Falta el 4>* pero nose si va aqui en java
    private SwerveDriveKinematics kinematics;
    private SwerveModulePosition odometryPos;
    private SwerveDrivePoseEstimator odometry;

    public SwerveChassis() {
        AHRS navAhrs;

        SwerveModule frontLeftModule;
        SwerveModule frontRightModule;
        SwerveModule backLeftModule;
        SwerveModule backRightModule;

        double wheelVoltage;
        double targetAngle;

        Translation2d modulePos; // Falta el 4>* pero nose si va aqui en java
        SwerveDriveKinematics kinematics;
        SwerveModulePosition odometryPos;
        SwerveDrivePoseEstimator odometry;

    }
    
    public void setModulePositions(Translation2d, positions){
        this.modulePos = positions;
        setModulePositions.kinematics = SwerveDriveKinematics(modulePos);
    }

    public void setModulesRatios(double turnRatio, double driveRatio, double wheelDiameter){
        frontRightModule.setGearRatio(turnRatio, driveRatio);
        frontLeftModule.setGearRatio(turnRatio, driveRatio);
        backRightModule.setGearRatio(turnRatio, driveRatio);
        backLeftModule.setGearRatio(turnRatio, driveRatio);

        frontRightModule.setWheelDiameter(wheelDiameter);
        frontLeftModule.setWheelDiameter(wheelDiameter);
        backRightModule.setWheelDiameter(wheelDiameter);
        backLeftModule.setWheelDiameter(wheelDiameter);
    };

    public void setModules(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight){
        this.frontLeftModule = frontLeft;
        this.frontRightModule = frontRight;
        this.backLeftModule = backLeft;
        this. backRightModule = backRight;
        
        odometryPos = new SwerveModulePosition[], 4> {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };

        odometry = new SwerveDrivePoseEstimator <4>{
            kinematics,
            Rotation2d{},
            odometryPos,
            Pose2d{}
        };
    }

    public void setRotatorPID(double kP, double kI, double kD){
        backRightModule.setRotatorPIDValues(kP, kI, kD);
        backLeftModule.setRotatorPIDValues(kP, kI, kD);
        frontRightModule.setRotatorPIDValues(kP, kI, kD);
        frontLeftModule.setRotatorPIDValues(kP, kI, kD);
    }

    public void setDrivePID(double kP, double kI, double kD){
        backRightModule.setDrivePIDValues(kP, kI, kD);
        backLeftModule.setDrivePIDValues(kP, kI, kD);
        frontRightModule.setDrivePIDValues(kP, kI, kD);
        frontLeftModule.setDrivePIDValues(kP, kI, kD);
    }

    public void setFeedForward(Voltage kS, Voltage kV, Voltage kA){
        backRightModule.setFFConstants(kS, kV, kA);
        backLeftModule.setFFConstants(kS, kV, kA);
        frontRightModule.setFFConstants(kS, kV, kA);
        frontLeftModule.setFFConstants(kS, kV, kA);
    }

    public void setUseRawVoltageSpeed(boolean set){
        frontLeftModule.setUseRawVoltageSpeed(set);
        frontRightModule.setUseRawVoltageSpeed(set);
        backLeftModule.setUseRawVoltageSpeed(set);
        backRightModule.setUseRawVoltageSpeed(set);
    }

    public void setTargetAngle(double targetAngle){
        this.targetAngle = targetAngle;
    }

    public void setSpeed(ChassisSpeeds speeds){
        this.linearX = speeds.vx.value();
        this.linearY = speeds.vy.value();
        this.angular = speeds.omega.value();

        SwerveModuleState [], 4> desiredStates = kinematics.ToSwerveModulesStates(speeds);

        setModulesStates(desiredStates);
    }

    public void setWheelVoltage(double voltage){
        frontLeftModule.setWheelVoltage(voltage);
        frontRightModule.setWheelVoltage(voltage);
        backLeftModule.setWheelVoltage(voltage);
        backRightModule.setWheelVoltage(voltage);
    }

    public void getOdometry(){
        return odometry.GetEstimatedPosition();
    }

    public void resetOdometry(Pose2d initPose){
        odometry.ResetPosition(Rotation2d.fromDegrees(navx.getAngle()), getModulePosition(), initPose);
    }

    double getHeadingRate(){
        return -navx.GetRate();
    }

    final SwerveDriveKinematics<4>& getKinematics(){
        return kinematics;
    }

    public void addVisionMeasurement(Pose2d pose, second_t timestamp){
        odometry.AddVisionMeasurement(pose, timestamp);
    }

    public void resetNavx(double angle){
        Pose2d.actualOdometry = getOdometry();
        Pose2d.newOdometry{actualOdometry.X(), actualOdometry.Y(), degree_t(angle)};
        resetOdometry(newOdometry);
    }

    public void setModuleStates(SwerveModuleState[], 4> desiredStates){
        frontLeftModule.setState(desiredStates[0]);
        frontRightModule.setState(desiredStates[1]);
        backRightModule.setState(desiredStates[2]);
        backLeftModule.setState(desiredStates[3]);

        backRightModule.setVoltages();
        backLeftModule.setVoltages();
        frontLeftModule.setVoltages();
        frontRightModule.setVoltages();
    }

    SwerveModuleState[], 4> getModuleStates(){
        SwerveModuleState[], 4> modulePositions{
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
        return modulePositions;
    }

    SwerveModulePosition[], 4> getModulePosition(){
        SwerveModulePosition[], 4> modulePositions{
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
        return modulePositions;
    }

    double getPitch(){
        return navx.GetPitch();
    }

    double getYaw(){
        return getOdometry().Rotation().Degrees().value();
    }

    double getRoll(){
        return navx.GetRoll();
    }

    public void updateOdometry(){
        odometry.Update(Rotation2d(degree_t(-navx.GetAngle())), getModulePosition());
    }

    public void shuffleboardPeriodic(){
        Auto estimatedPos = getOdometry();
        SmartDashboard.PutNumber("Roll", getRoll());

        SmartDashboard.PutNumber("OdometryX", estimatedPos.X().value());
        SmartDashboard.PutNumber("OdometryY", estimatedPos.Y().value());
        SmartDashboard.PutNumber("AnglenaveX", estimatedPos.Rotation().Degrees().value());
    }
}
