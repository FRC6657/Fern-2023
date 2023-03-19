// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.PivotConstants.State;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private final CANSparkMax mMotor;
  private final ProfiledPIDController mPID;

  private final RelativeEncoder mNeoEncoder;
  private final DutyCycleEncoder mTBEncoder;

  private State mCurrentState;

  
  public double mTargetAngle;
  public double mNeoOffset;


  public Pivot() {
    mMotor = new CANSparkMax(7, MotorType.kBrushless);
    mPID = new ProfiledPIDController(0.01, 0, 0, new Constraints(540, 540*8));
    mMotor.setSmartCurrentLimit(15);
    mMotor.setInverted(false);
    mMotor.enableVoltageCompensation(RobotConstants.maxVoltage);
    mMotor.burnFlash();
    mMotor.setIdleMode(IdleMode.kBrake);

    mTBEncoder = new DutyCycleEncoder(9);
    mTBEncoder.setPositionOffset(PivotConstants.kThroughboreOffset);


    mNeoEncoder = mMotor.getEncoder();
    mTargetAngle = State.STARTING.angle;
    mNeoEncoder.setPositionConversionFactor(PivotConstants.kPositionConversion);
    mNeoEncoder.setVelocityConversionFactor(PivotConstants.kVelocityConversion);

    Timer.delay(1);

    zeroEncoder();
    mCurrentState = State.STARTING;

  }

  public boolean atTarget() {

    double tolerance = 2; 

    return (Math.abs(mNeoEncoder.getPosition() - mTargetAngle) < tolerance);

}

  public Command changeState(State state){
    return new InstantCommand(() -> mCurrentState = state);
  }

  public void set(double percent){
    mMotor.set(percent);
  }

  public void runPivot(){
    mPID.setGoal(mTargetAngle);
    mMotor.set(mPID.calculate(mNeoEncoder.getPosition()));
  }

  public double getThroughBoreAngle() {
    return ((mTBEncoder.getAbsolutePosition()) - mTBEncoder.getPositionOffset()) * 360;
  }

public void zeroEncoder() {
  mNeoOffset = getThroughBoreAngle();
}


  @Override
  public void periodic() {
    SmartDashboard.putNumber("TBE Raw", mTBEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("TBE Degrees", getThroughBoreAngle());
    SmartDashboard.putNumber("Neo Degrees", mNeoEncoder.getPosition() + mNeoOffset);
    SmartDashboard.putNumber("Neo Offset", mNeoOffset);
    SmartDashboard.putBoolean("At Setpoint", mPID.atSetpoint());
    SmartDashboard.putNumber("Motor Voltage", mMotor.get() * 12);
    SmartDashboard.putNumber("Set Point", mPID.getGoal().position);
  }
}
