// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public static class RobotConstants{
        public static double maxVoltage = 15;
    }

    public static class IntakeConstants {
    
        public static final double kInSpeed = 0.9;
        public static final double kOutSpeed = 0.9;
    
        public static final double kS = 1;
    
        public static enum State {
          GRAB(-kInSpeed),
          RELEASE(kOutSpeed),
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
        public static double kThroughboreOffset = 0.149;

        
    

        public static enum State {
          SUBSTATION(0),
          L1(0),
          L2(0),
          STOP(0),
          STARTING(0),
          CARRY(0);
    
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
