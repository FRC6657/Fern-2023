// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

  public static class DriveConstants{
    public static double kTurboForwardSpeed = 1.0;
    public static double kNormalForwardSpeed = 0.4;
    public static double kTurboTurningSpeed = 0.5;
    public static double kNormalTurningSpeed = 0.5;

    public static enum FrontState {
      FORWARD(1),
      REVERSE(-1);

      public final double direction;

      /**
       * @param direction Motor Percentage
       */
      FrontState(double direction) {
        this.direction = direction;
      }

    }

    public static enum RotateState {
      POSITVIE(1),
      NEGATIVE(-1);

      public final double direction;

      /**
       * @param direction Motor Percentage
       */
      RotateState(double direction) {
        this.direction = direction;
      }

    }


  }

    public static class RobotConstants{
        public static double maxVoltage = 15;
        public static class CAN{
          public static int kFrontLeft = 1;
          public static int kBackLeft = 2;
          public static int kFrontRight = 3;
          public static int kBackRight = 4;
          public static int kPigeon = 5;
          public static int kIntake = 6;
          public static int kPivot = 7;

        }
    }

    public static class IntakeConstants {
    
        public static final double kInSpeed = 0.35;
        public static final double kOutSpeed = 0.99;
    
        public static final double kS = 0;
    
        public static enum State {
          GRAB(-kInSpeed),
          RELEASE(kOutSpeed),
          L1RELEASE(0.30),
          IDLE(-kS/12),
          STOP(0),
          STARTING(0);
    
          public final double speed;
    
          /**
           * @param speed Motor Percentage
           */
          State(double speed) {
            this.speed = speed;
          }
    
        }
    
      }

      public static class PivotConstants {

        public static double kGearing = ((1.0/15)*(16.0/60));
        public static double kVelocityConversion = kGearing*(1/60.0)*360;
        public static double kPositionConversion = kGearing*360;
        public static double kThroughboreOffset = 0.475;

        
    

        public static enum State {
          SUBSTATION(5),
          L1(45),
          L2(-20),
          STOP(0),
          STARTING(-80),
          FLOOR(110),
          CARRY(-80);
    
          public final double angle;
    
          /**
           * @param angle Pivot Angle
           */
          State(double angle) {
            this.angle = angle;
          }
    
        }
    
      }
}
