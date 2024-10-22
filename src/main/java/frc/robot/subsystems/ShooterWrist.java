// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class ShooterWrist extends SubsystemBase {

  public CANSparkMax shooterWristMotor;
  public AbsoluteEncoder m_AbsoluteEncoder;

  private SparkPIDController shooterWristPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  //private double maxSpeed = 0.25;
  //private double deadBand = 0.1;

  /** Creates a new ShooterWrist. */
  public ShooterWrist() {
    shooterWristMotor = new CANSparkMax(Constants.RobotConstants.shooterWristCANID, MotorType.kBrushless);
    shooterWristMotor.restoreFactoryDefaults();

    shooterWristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    shooterWristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    shooterWristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    shooterWristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    shooterWristPIDController = shooterWristMotor.getPIDController();
    m_AbsoluteEncoder = shooterWristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterWristPIDController.setFeedbackDevice(m_AbsoluteEncoder);


    //m_AbsoluteEncoder.setPositionConversionFactor(360);
    //m_AbsoluteEncoder.setVelocityConversionFactor(1);
    shooterWristMotor.setInverted(false);
    shooterWristMotor.setIdleMode(IdleMode.kBrake);

    shooterWristMotor.setSmartCurrentLimit(45);

    kP = 7; //3.8 last working value
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = -2.5;
    kMaxOutput = .5;
    kMinOutput = -.5;

    shooterWristPIDController.setP(kP);
    shooterWristPIDController.setI(kI);
    shooterWristPIDController.setD(kD);
    shooterWristPIDController.setIZone(kIz);
    shooterWristPIDController.setFF(kFF);
    shooterWristPIDController.setOutputRange(kMinOutput, kMaxOutput);

    shooterWristPIDController.setPositionPIDWrappingEnabled(true);
    shooterWristPIDController.setPositionPIDWrappingMinInput(0);
    shooterWristPIDController.setPositionPIDWrappingMaxInput(1);
  }

  @Override

  public void periodic() {
    double position = limelightGetShooterAngle();
    SmartDashboard.putNumber("limelight shooter", position);
    SmartDashboard.putNumber("Wrist Encoder", getAbsoluteEncoderPosition());
  }

  public void setSpeed(double speed)
  {
    shooterWristMotor.set(speed);
  }

  public void stop()
  {
    shooterWristMotor.stopMotor();
  }

  public double getAbsoluteEncoderPosition() {
    return m_AbsoluteEncoder.getPosition();
  }

  public void setPosition(double position) {
    shooterWristPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  // Method to adjust our shooter wrist angle based on distance from speaker
  // Returns the encoder value to set the position of the wrist
  public double limelightGetShooterAngle() {
    double distance = RobotContainer.limelight.getDistanceToTarget();
    // if (LimelightHelpers.getTV("limelight"))
    // {
    //   return (5.28487/(distance + 17.3769)) + 0.179797;
    // }
    // else
    // {
    //   return 0.286;
    // }
    return (4874.07/(distance - 3226.24)) + 2.15252;
  }
}

