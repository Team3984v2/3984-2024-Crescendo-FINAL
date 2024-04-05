// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Swerve{

        public static final double stickDeadband = 0.1;
        public static final boolean invertGyro = false;//true;

        /* Drivetrain Constants */
        public static final double trackWidth = .5543;
        public static final Translation2d f1ModuleOffset = new Translation2d(trackWidth/2, trackWidth/2);
        public static final Translation2d f2ModuleOffset = new Translation2d(trackWidth/2, -trackWidth/2);
        public static final Translation2d b1ModuleOffset = new Translation2d(-trackWidth/2, trackWidth/2);
        public static final Translation2d b2ModuleOffset = new Translation2d(-trackWidth/2, -trackWidth/2);
        public static final double wheelBase = Units.inchesToMeters(21.73);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter*Math.PI;
        
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); 
        public static final double angleGearRatio = ((150.0 / 7.0)/1.0); 

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.005;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.025;
        public static final double driveKI = 0;
        public static final double driveKD = 0;
        public static final double driveKFF = 0.0;

        /* TODO Drive Motor Characterization Values */
        public static final double driveKS = 0.11563;//0.667;//0.11563;
        public static final double driveKV = 2.056;//2.44;//2.656;
        public static final double driveKA = 0.276;//0.27;//0.276;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false; //true
        public static final boolean angleInvert = true;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; //public static final boolean canCoderInvert = false;
        
        
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //Was module2
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(287.490234375);//337.02734375
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

        /* Back Left Module - Module 1 */
        public static final class Mod2 { //was module0
            public static final int driveMotorID = 17;
            public static final int angleMotorID = 16;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(326.953125);//508.9);//328.9);//149.15
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 2 */
        public static final class Mod1 { //Was module3
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(123.57421875);//219.2//40.0
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // was module 1
            public static final int driveMotorID = 15;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(249.609375);//23.55//203.75
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

        

        public static final class flywheel {
            public static final double FWDiameter = 4.0; //Inches NEED TO CONVERT
            public static final double tolerance = 1; //Meters per second
            public static final class FWtop {//TODO PID
                public static final int FWid = 18;
                public static final double kP = 0.1;//0.1;
                public static final double kI = 0;
                public static final double kD = 0;
                public static final double kV = 0.01;
                public static final double gearRatio = 1; 
            }
            public static final class FWbott { //TODO PID
                public static final int FWid = 19;
                public static final double kP = 0.1;//0.1;
                public static final double kI = 0;
                public static final double kD = 0;
                public static final double kV = 0.01;
                public static final double gearRatio = 1; 
            }
            // AMP and SPEAKER velocity presets
            public static final double AMP = 10;
            public static final double SPEAKER = 15;//30;

        }
        public static final class climber{
            public static final int rotMotorID = 8; // climber left
            public static final int rotMotorID2 = 20; // climber right
            public static final double tolerance = 1.0;
            public static final double kP = 0.01;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double gearRatio = 25;
            public static final Rotation2d[] FULLEXTENSION= new Rotation2d[]{Rotation2d.fromDegrees(-2022), Rotation2d.fromDegrees(2022)};
        }
        public static final class intake{
            public static final double tolerance = 0.1;
            public static final class intakeArm{
                public static final int rotMotorID = 12; //TODO
                public static final double kP = 0.009;
                public static final double kI = 0;
                public static final double kD = 0.0001;
                public static final double gearRatio = 100;

            }
            public static final class intakeMotor{
                public static final int IDR = 0;
                public static final int IDL = 1;

            }
            public static final Rotation2d INTAKE = Rotation2d.fromDegrees(-200);
            public static final Rotation2d AMPSHOT = Rotation2d.fromDegrees(-94+10+5);
            public static final Rotation2d IDLE = Rotation2d.fromDegrees(0);
            

        }
        
        public static final class camera{
            public static final double Height = 0;
            //public static final double TargetHeight = 0;
            public static final double CameraPitch = 0;
            public static final Rotation3d camAngle = new Rotation3d(0, 0, 0/*1.571*/);
            public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, 0, 0.2921), camAngle);
            public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
        }


    }

    public static final class AutoConstants {
        public static final String kAutoKey = "AutonSelection";
        public static final String defaultAuto = "1";
        
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.000000000000001;//0.2;//0.1;
        public static final double kPYController = 0.000000000000001;//0.2;// 0.1;
        public static final double kPThetaController = 7.5;//7.5
        public static final Translation2d GOALRIGHT = new Translation2d(1, -1);
        public static final Translation2d GOALLEFT = new Translation2d(1, 1);
        public static final Translation2d GOALMIDDLE = new Translation2d(1, 0);
        public static final PathConstraints constraints = new PathConstraints(kMaxAngularSpeedRadiansPerSecond, kMaxAccelerationMetersPerSecondSquared, 1, 1);//TODO change
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
