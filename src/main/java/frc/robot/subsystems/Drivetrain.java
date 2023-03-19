// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Drivetrain extends SubsystemBase {
  private final WPI_TalonSRX mFrontLeft;
  private final WPI_TalonSRX mFrontRight;
  private final WPI_TalonSRX mBackLeft;
  private final WPI_TalonSRX mBackRight;
  private final WPI_PigeonIMU mPigeon;
  private final DifferentialDrive mDifferentialDrive;
  public Drivetrain() {
    mFrontLeft = new WPI_TalonSRX(1);
    mFrontRight = new WPI_TalonSRX(2);
    mBackLeft = new WPI_TalonSRX(3);
    mBackRight = new WPI_TalonSRX(4);
    mPigeon = new WPI_PigeonIMU(5);
    mBackLeft.follow(mFrontLeft);
    mBackRight.follow(mFrontRight);
    mFrontLeft.configVoltageCompSaturation(RobotConstants.maxVoltage);
    mFrontRight.configVoltageCompSaturation(RobotConstants.maxVoltage);
    mBackLeft.configVoltageCompSaturation(RobotConstants.maxVoltage);
    mBackRight.configVoltageCompSaturation(RobotConstants.maxVoltage);
    mFrontLeft.enableVoltageCompensation(true);
    mFrontRight.enableVoltageCompensation(true);
    mBackLeft.enableVoltageCompensation(true);
    mBackRight.enableVoltageCompensation(true);
    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);

    mDifferentialDrive = new DifferentialDrive(mFrontLeft, mFrontRight);

    SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 40, 40, 0);
    mFrontLeft.configSupplyCurrentLimit(currentLimit);
    mFrontRight.configSupplyCurrentLimit(currentLimit);
    mBackLeft.configSupplyCurrentLimit(currentLimit);
    mBackRight.configSupplyCurrentLimit(currentLimit);


    

  }

  public double getAngle(){
    return mPigeon.getRotation2d().getDegrees();
  }

  public void drive(double xSpeed, double rSpeed, boolean turnInPlace){
    mDifferentialDrive.curvatureDrive(xSpeed, rSpeed, turnInPlace);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
