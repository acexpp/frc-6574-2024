// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterWrist extends SubsystemBase {

  public CANSparkMax shooterWristMotor;
  public AbsoluteEncoder m_AbsoluteEncoder;
  //private RelativeEncoder wristEncoder;

  private SparkPIDController shooterWristPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  //private double maxSpeed = 0.25;
  //private double deadBand = 0.1;

  /** Creates a new ShooterWrist. */
  public ShooterWrist() {
    shooterWristMotor = new CANSparkMax(Constants.RobotConstants.shooterWristCANID, MotorType.kBrushless);
    shooterWristMotor.restoreFactoryDefaults();

    shooterWristPIDController = shooterWristMotor.getPIDController();
    m_AbsoluteEncoder = shooterWristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterWristPIDController.setFeedbackDevice(m_AbsoluteEncoder);


    //m_AbsoluteEncoder.setPositionConversionFactor(360);
    //m_AbsoluteEncoder.setVelocityConversionFactor(1);
    shooterWristMotor.setInverted(false);
    shooterWristMotor.setIdleMode(IdleMode.kBrake);

    shooterWristMotor.setSmartCurrentLimit(45);

    kP = 5; //3.8 last working value
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = -0.5;
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
    SmartDashboard.putNumber("Shooter Wrist Position", getAbsoluteEncoderPosition());
    double position = limelightGetShooterAngle();
    SmartDashboard.putNumber("limelight shooter", position);
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
    //double heightShooterToSpeaker = 54.0 + (0.2 * (RobotContainer.limelight.getDistanceToTarget() - 45.5));
    /*
    double heightShooterToSpeaker = 54.0 + (0.005 * Math.pow((RobotContainer.limelight.getDistanceToTarget() - 45.5),2));
    double shooterAngleDegrees = Units.radiansToDegrees(Math.atan(heightShooterToSpeaker/distance));
    double shooterOffset = 45.36;
    return (shooterAngleDegrees + shooterOffset)/360;
    */
    return (1.79863/(distance - 12.1876)) + 0.204771;
  }
}

