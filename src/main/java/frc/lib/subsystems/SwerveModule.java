// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.enums.NeutralMode.ControllerNeutralMode;
import frc.lib.motorcontrollers.OverTalonFX;
import frc.lib.sensors.OverCANCoder;

public class SwerveModule extends SubsystemBase {

  private OverTalonFX m_driveMotor;
  private OverTalonFX m_turningMotor;
  private OverCANCoder m_CanCoder;

  private SimpleMotorFeedforward m_FeedForward;

  private SwerveModuleState m_State = new SwerveModuleState();

  private double m_wheelDiameter = 1;
  private double m_driveMotorGearRatio = 1;
  private double m_turningMotorGearRatio = 1;

  private String m_name;

  /** Creates a new SwerveModule. */
  public SwerveModule(int rotatorID, int wheelID, int canCoderID, double offset, String moduleName, String canBus) {
    m_driveMotor = new OverTalonFX(wheelID, ControllerNeutralMode.BRAKE, true, canBus);
    m_turningMotor = new OverTalonFX(rotatorID, ControllerNeutralMode.BRAKE, true, canBus);
    m_CanCoder = new OverCANCoder(canCoderID, offset, canBus);

    m_turningMotor.setContinousWrap();
    m_turningMotor.setFusedCANCoder(canCoderID);
    m_turningMotor.setClosedLoopVoltageRamp(0.1);
    m_turningMotor.SetSupplyCurrentLimit(true, 20, 30, 0.5);
    m_turningMotor.setPositionVoltage(0, false);

    m_driveMotor.zeroPosition();
    m_driveMotor.setClosedLoopVoltageRamp(0.1);
    m_driveMotor.SetSupplyCurrentLimit(true, 20, 30, 0.5);

    m_name = moduleName;
    m_name = m_name.trim();
  }

  public SimpleMotorFeedforward get_FeedForward() {
    return m_FeedForward;
  }

  public void setRotatorPIDValues(double kP, double kI, double kD) {
    m_turningMotor.setPIDValues(kP, kI, kD, 0, 0);

  };

  public void setDrivePIDValues(double kP, double kI, double kD) {
    m_driveMotor.setPIDValues(kP, kI, kD, 0, 0);
  };

  public void setFFConstants(double ks, double kv, double ka) {
    m_FeedForward = new SimpleMotorFeedforward(ks, kv, ka);
  }

  public void setGearRatio(double turn, double wheel) {
    m_turningMotorGearRatio = turn;
    m_driveMotorGearRatio = wheel;
    m_turningMotor.setRotorToSensorRatio(m_turningMotorGearRatio);
    m_driveMotor.setRotorToSensorRatio(m_driveMotorGearRatio);
  }

  public void setWheelDiameter(double wheelDiameter) {
    m_wheelDiameter = wheelDiameter;
  }

  public double getSpeed() {
    return m_driveMotor.getVelocity(m_wheelDiameter, m_driveMotorGearRatio);
  }

  public double setSpeed(double speed) {
    return ((speed / (m_wheelDiameter * Math.PI)));
  }

  public double getDistance() {
    return m_driveMotor.getDistance(m_wheelDiameter, m_driveMotorGearRatio);
  }

  public double getAngle() {
    return m_CanCoder.getSensorAbsolutePosition();

  }

  public SwerveModuleState getState() {
    SwerveModuleState state = new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAngle()));

    // state.speedMetersPerSecond = getSpeed();
    // state.angle = Rotation2d.fromDegrees(getAngle());

    return state;
  };

  public void setState(SwerveModuleState state) {
    m_State = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));
  };

  public SwerveModulePosition getPosition() {
    SwerveModulePosition position = new SwerveModulePosition();

    position.angle = Rotation2d.fromDegrees(getAngle());
    position.distanceMeters = getDistance();

    return position;
  };

  public void setVoltages() {
    m_turningMotor.setPositionVoltage(m_State.angle.getDegrees() / 360.0, false);

    m_driveMotor.setVoltage(setSpeed(m_State.speedMetersPerSecond), false);

  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber(m_name + "/Speed", getSpeed());
    // SmartDashboard.putNumber(m_name + "/Target", m_State.angle.getDegrees());
    // SmartDashboard.putNumber(m_name + "/Angle", getAngle());
  }
}

// hola eric
