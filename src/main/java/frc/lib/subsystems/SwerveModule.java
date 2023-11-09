// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.motorcontrollers.OverTalonFX;
import frc.lib.sensors.OverCANCoder;

public class SwerveModule extends SubsystemBase {

  OverTalonFX m_driveMotor;
  OverTalonFX m_turningMotor;
  OverCANCoder m_CanCoder;

  SimpleMotorFeedforward m_feedForward;

  SwerveModuleState m_state;

  double m_wheelDiameter = 1;

  boolean useRawVoltageSpeed = false;

  /** Creates a new SwerveModule. */
  public SwerveModule(int rotatorID, int wheelID, int canCoderID, double offset, String moduleName, String canBus) {
    m_driveMotor = new OverTalonFX(canCoderID, null, false, offset, canBus);
    m_turningMotor = new OverTalonFX(canCoderID, null, false, offset, canBus);
    m_CanCoder = new OverCANCoder(canCoderID, offset, canBus);

    m_turningMotor.setContinousWrap();
    m_turningMotor.setFusedCANCoder(canCoderID);
    m_turningMotor.setClosedLoopVoltageRamp(0.1);
    m_turningMotor.SetSupplyCurrentLimit(true, 20, 30, 0.5);
    m_turningMotor.setPositionVoltage(0, false);

    m_driveMotor.zeroPosition();
    m_driveMotor.setClosedLoopVoltageRamp(0.1);
    m_driveMotor.SetSupplyCurrentLimit(true, 20, 30, 0.5);
  }

  public void setRotatorPIDValues(double kP, double kI, double kD) {
    m_turningMotor.setPIDValues(kP, kI, kD, 0, 0);

  };

  public void setDrivePIDValues(double kP, double kI, double kD) {
    m_driveMotor.setPIDValues(kP, kI, kD, 0, 0);
  };

  public void setFFConstants(double ks, double kv, double ka) {
    m_feedForward = new SimpleMotorFeedforward(ks, kv, ka);
  }

  public void setGearRatio(double turn, double wheel) {
    m_turningMotor.setRotorToSensorRatio(turn);
    m_driveMotor.setSensorToMechanismRatio(wheel);
  }

  public void setWheelDiameter(double wheelDiameter) {
    m_wheelDiameter = wheelDiameter;
  }

  public double getSpeed() {
    return m_driveMotor.getVelocity(m_wheelDiameter);
  }

  public double setSpeed(double speed) {
    return (speed / ((m_wheelDiameter * Math.PI)));
  }

  public double getDistance() {
    return m_driveMotor.getDistance(m_wheelDiameter);
  }

  public double getAngle() {
    return m_CanCoder.getSensorAbsolutePosition() * 360.0;
  }

  public SwerveModuleState getState() {
    SwerveModuleState state = new SwerveModuleState();

    state.speedMetersPerSecond = getSpeed();
    state.angle = Rotation2d.fromDegrees(getAngle());

    return state;
  };

  public void setState(SwerveModuleState state) {
    m_state = SwerveModuleState.optimize(state, m_state.angle);
  };

  public SwerveModulePosition getPosition() {
    SwerveModulePosition state = new SwerveModulePosition();
    // double meter_t = getDistance();
    // double degree_t = getAngle();
    state.angle = Rotation2d.fromDegrees(getAngle());
    state.distanceMeters = getDistance();
    // SwerveModulePosition position = new getDistance(), getAngle();

    return state;
  };

  public SwerveModule setWheelVoltage(double wheelVoltage) {
    SwerveModule state = new SwerveModule(0, 0, 0, wheelVoltage, getSubsystem(), getName());
    state.setWheelVoltage(wheelVoltage);
    return state;
  };

  public void setUseRawVoltageSpeed(boolean set) {
    useRawVoltageSpeed = set;
  };

  public SwerveModule setVoltages() {
    m_turningMotor.setPositionVoltage(m_state.angle.getDegrees() / 360.0, false);

    if (useRawVoltageSpeed) {
      m_driveMotor.setVoltage(wheelVoltage, true);
    } else {
      m_driveMotor.setVoltage(m_wheelDiameter, useRawVoltageSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
