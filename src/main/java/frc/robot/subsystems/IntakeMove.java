// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMove extends SubsystemBase {

  public CANSparkMax intakeMoveLeft;

  public CANSparkMax intakeMoveRight;
  //private RelativeEncoder wristEncoder;

  private SparkLimitSwitch intakeReverseLimit;
  private SparkPIDController intakeMoveLeftPidController;
  private SparkPIDController intakeMoveRightPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  //private double maxSpeed = 0.25;
  //private double deadBand = 0.1;

  /** Creates a new IntakeMove. */
  public IntakeMove() {
    intakeMoveLeft = new CANSparkMax(Constants.RobotConstants.intakeMoveLeftCANID, MotorType.kBrushless);
    intakeMoveRight = new CANSparkMax(Constants.RobotConstants.intakeMoveRightCANID, MotorType.kBrushless);

    //intakeReverseLimit = intakeMoveLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    intakeMoveLeft.restoreFactoryDefaults();
    intakeMoveRight.restoreFactoryDefaults();

    intakeMoveLeft.setIdleMode(IdleMode.kBrake);
    intakeMoveRight.setIdleMode(IdleMode.kBrake);

    intakeMoveLeft.setSmartCurrentLimit(30);
    intakeMoveRight.setSmartCurrentLimit(30);

    intakeMoveLeft.setInverted(true);
    intakeMoveRight.setInverted(false);

    //intakeMoveRight.follow(intakeMoveLeft, true);

    //intakeMoveLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    //intakeMoveLeft.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, maxIntakeExtension);

    intakeMoveLeft.getEncoder().setPosition(0);
    intakeMoveRight.getEncoder().setPosition(0);

    intakeMoveLeftPidController = intakeMoveLeft.getPIDController();
    intakeMoveLeft.getEncoder();

    intakeMoveRightPidController = intakeMoveRight.getPIDController();
    intakeMoveRight.getEncoder();

    kP = 7; //2.5 last working value
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.11;
    kMaxOutput = .2;
  
    kMinOutput = -.2;

    intakeMoveLeftPidController.setP(kP);
    intakeMoveLeftPidController.setI(kI);
    intakeMoveLeftPidController.setD(kD);
    intakeMoveLeftPidController.setIZone(kIz);
    intakeMoveLeftPidController.setFF(kFF);
    intakeMoveLeftPidController.setOutputRange(kMinOutput, kMaxOutput);

    intakeMoveLeftPidController.setPositionPIDWrappingEnabled(true);
    intakeMoveLeftPidController.setPositionPIDWrappingMinInput(0);
    intakeMoveLeftPidController.setPositionPIDWrappingMaxInput(1);

    intakeMoveRightPidController.setP(kP);
    intakeMoveRightPidController.setI(kI);
    intakeMoveRightPidController.setD(kD);
    intakeMoveRightPidController.setIZone(kIz);
    intakeMoveRightPidController.setFF(kFF);
    intakeMoveRightPidController.setOutputRange(kMinOutput, kMaxOutput);

    intakeMoveRightPidController.setPositionPIDWrappingEnabled(true);
    intakeMoveRightPidController.setPositionPIDWrappingMinInput(0);
    intakeMoveRightPidController.setPositionPIDWrappingMaxInput(1);
  }

  @Override

  public void periodic() {
    SmartDashboard.putNumber("Intake Position Left", getAbsoluteEncoderPositionLeft());
    SmartDashboard.putNumber("Intake Position Right", getAbsoluteEncoderPositionRight());
    //SmartDashboard.putNumber("Wrist Joystick", RobotContainer.operator.getRawAxis(5));
    //SmartDashboard.putNumber("Wrist encoder", wristMotor.getEncoder().getPosition());

    /* if (RobotContainer.operator.getRawAxis(5) > deadBand) {
      wristMotor.set(-RobotContainer.operator.getRawAxis(5) * maxSpeed);
    } else if (RobotContainer.operator.getRawAxis(5) < -deadBand) {
      wristMotor.set(-RobotContainer.operator.getRawAxis(5) * maxSpeed);
    } */



    /* if (RobotContainer.operator.getRawButtonPressed(1)) {
      intakeMotor.set(1);
    }
      else if (RobotContainer.operator.getRawButtonReleased(1)) {
        intakeMotor.set(0);
      }

      if (RobotContainer.operator.getRawButtonPressed(2)) {
        intakeMotor.set(-1);
      }
      else if (RobotContainer.operator.getRawButtonReleased(2)) {
        intakeMotor.set(0);
      } */


    }




  public void setSpeed(double speed)
  {
    intakeMoveLeft.set(speed);
    intakeMoveRight.set(speed);
  }

  public void stop()
  {
    intakeMoveLeft.stopMotor();
    intakeMoveRight.stopMotor();
  }


  public double getAbsoluteEncoderPositionLeft() {
    return intakeMoveLeft.getAbsoluteEncoder().getPosition();
  }

  public double getAbsoluteEncoderPositionRight() {
    return intakeMoveRight.getAbsoluteEncoder().getPosition();
  }

  public void setPosition(double positionLeft, double positionRight) {
    intakeMoveLeftPidController.setReference(positionLeft, CANSparkMax.ControlType.kPosition);
    intakeMoveRightPidController.setReference(positionRight, CANSparkMax.ControlType.kPosition);
  }

}
