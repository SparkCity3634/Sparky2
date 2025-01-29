// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {

  public static final int BackRightDriveCANId = 41;
  public static final int BackRightTurnCANId = 42;
  public static final int FrontRightDriveCANId = 11;
  public static final int FrontRightTurnCANId = 12;
  public static final int FrontLeftDriveCANId = 21;
  public static final int FrontLeftTurnCANId = 22;
  public static final int BackLeftDriveCANId = 31;
  public static final int BackLeftTurnCANId = 32;

    public static final String canBusName = "rio";

   public static final int FrontRightTurnCancoderId = 10;
   public static final int FrontLeftTurnCancoderId = 20;
   public static final int BackLeftTurnCancoderId = 30;
   public static final int BackRightTurnCancoderId = 40;

  
    
  }

  public static class PIDConstants {


  public static final double maxRPM;
  public static final double setTurn;
  public static final double kdP =  0.4;
   public static final double kdI = 0.00000;
   public static final double kdD = 0.05;
   public static final double kdIz = 0;
   public static final double kdFF = 0.01;
   public static final double kdMaxOutput = 1; 
   public static final double kdMinOutput = -1;
   public static final double ktP = 0.07;
   public static final double ktI = 0.0000;
   public static final double ktD = 0.001; 
   public static final double ktIz = 0; 
   public static final double ktFF = 0.003; 
   public static final double ktMaxOutput = .9; 
   public static final double ktMinOutput = -.9;

   /* For shooters
   public static final double ksP;
   public  static final double ksI;
   public  static final double ksD;
   public  static final double ksIz;
   public  static final double ksFF;
   public  static final double ksMaxOutput;
   public  static final double ksMinOutput;
   */
   /*
   public static final double xAxis;
   public  static final double yAxis; 
   public  static final double kYawRate; 

   public  static final double o_lyAxis;
   public  static final double o_ryAxis;
   */
  public  static final double maxVel = 4;
   public  static final double maxYaw = 2*Math.PI;
   public static final double k_posConv = .048; // linear meters traveled at wheel, per motor rotation
   public static final double k_velConv = .0008106; // linear meters per second (m/s) speed at wheel, convert from motor RPM
   public static final double k_turnConv = 16.8; //wheel degrees per motor rotation from steering relative encoder
   public static final double rotations;
   
   //Create Swerve Kinematics modules

   
public static final Translation2d m_BackLeftLocation = new Translation2d(-0.25 ,0.250);
public static final Translation2d m_BackRightLocation = new Translation2d(-0.25 ,-.250);
public static final Translation2d m_FrontLeftLocation = new Translation2d(.250,.250);
public static final Translation2d m_FrontRightLocation = new Translation2d(.250,-.250);

public static final int SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
public static final int SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
public static final int SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  }
}
