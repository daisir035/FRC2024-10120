package frc.robot;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final class OIConstants {
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
    
        public static final double stickDeadband = 0.04;
    
      }

    public static final class Swerve {

        public static final int pigeonID = 0;

        /* Drivetrain Constants */
        public static final double trackWidth = 0.408; //TODO: This must be tuned to specific robot
        public static final double wheelBase = 0.558; //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = 0.096*Math.PI;
        

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = 6.75;
        public static final double angleGearRatio = 18.0;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 47.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.3;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.01; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0;
        public static final double driveKF = 0;
    
        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.21628; //TODO: This must be tuned to specific robot
        public static final double driveKV = 3.2;
        public static final double driveKA = 0.09134;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.7; //TODO: This must be tuned to specific robot
        public static final double maxModuleSpeed = 4.7;

        
        /** Meters per Second Squared */
        public static final double maxAcceleration = 3.5; //TODO: This must be tuned to specific robot
        public static final double maxDeceleration = 20.0;
        
        public static final double maxCentripetalAcceleration=100;

        /** Radians per Second Squared */
        public static final double maxAngularAcceleration = 10; //TODO: This must be tuned to specific robot
        public static final double maxAngularDeceleration = 30; //TODO: This must be tuned to specific robot


        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final InvertedValue angleInvertedValue =InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveInvertedValue =InvertedValue.CounterClockwise_Positive;
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-162+180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset,
                InvertedValue.CounterClockwise_Positive,InvertedValue.CounterClockwise_Positive,SensorDirectionValue.CounterClockwise_Positive);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-26+180);//180-26.19
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset,
                InvertedValue.CounterClockwise_Positive,InvertedValue.CounterClockwise_Positive,SensorDirectionValue.CounterClockwise_Positive);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-176+180);//0.8789
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset,
                InvertedValue.CounterClockwise_Positive,InvertedValue.CounterClockwise_Positive,SensorDirectionValue.CounterClockwise_Positive);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(174-180);//-0.8789
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset,
                InvertedValue.CounterClockwise_Positive,InvertedValue.CounterClockwise_Positive,SensorDirectionValue.CounterClockwise_Positive);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final double kPTranslationController = 5;
        public static final double kPRotationController = 5;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class CanID {
        public static final int intakeID = 13;
        public static final int feederID = 14;
        public static final int UpShootingID = 16;
        public static final int DownShootingID = 15;
        public static final int elevatorID = 17;
    
        public static final int up_OptoeSwitchID = 7;
        public static final int down_OptoeSwitchID = 8;

        public static final int servoID = 6; 
        
      }

    public static class  IntakeConstant {
        public static final double KP = 0.7;
        public static final double KI = 0.0;
        public static final double KD = 0.03;
        public static final double KS = 0.14185;
        public static final double KV = 0.13;
        public static final double KA = 0.30801;
    
        public static final double spitVel = -80.0;
        public static final double suctionVel = 80.0;
        public static final double autoVel = 80.0;
        
      }

    public static class FeedingConstant{
        public static final double KP = 0.7;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.14185;
        public static final double KV = 0.13;
        public static final double KA = 0.30801;

        public static final double spitVel = 13.0;
        public static final double suctionVel = -13.0; //ds
        public static final double shootingVel = -25.0;
        public static final double autoVel = -15.0;
      }

    public static class ShootingConstant {
        public static final double KP = 15.7;
        public static final double KI = 0.0;
        public static final double KD = 0.03;
        public static final double KS = 0.14185;
        public static final double KV = 0.13;
        public static final double KA = 0.30801;
        
        public static final double lowPositionVel_Up = 50;
        public static final double lowPositionVel_Down = 50.0;

        public static final double hightPositionVel_Up = 30.0;
        public static final double hightPositionVel_Down = 76.0;

        public static final double armPOsitionVel_Up = 3.0;
        public static final double armPOsitionVel_Down = 28.0;

        public static final double passPOsitionVel_Up = 80.0;
        public static final double passPOsitionVel_Down = 30.0;

        public static final double defaultVel_Up = 20.0;
        public static final double defaultVel_Down = 20.0;
        
        public static final double autoVel = 35.0;
        public static final double errVel_auto = 0.5217;
        public static final double autoer  = 0 ;
      }

    public static class  ElevatorConstant {
        public static final double KP = 0.7;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KS = 0.14185;
        public static final double KV = 0.13;
        public static final double KA = 0.30801;
        public static final double KG = 0.30801;
        public static final double Vel = 50.0;
        public static final double Acc = 75.0;
        public static final double Jer = 0.0;
    
        public static final double height_Up = 1.0;
        public static final double height_Release  = 95.0;
        
        public static final double TestUPSpeed = 0.3;
        public static final double TestReleaseSpeed = -0.3;
    
        public static final double releaseAngle = 0.35;
        public static final double upAngle = 0.0;

        public static final double MaxHeight = 95.0;
        public static final double MinHeight = 0.0;

        
      }
      



    public static class FieldConstants{
            //the position of each, robot can use this to calculate its current position
        public static final List<Pose3d> aprilTagPoses=List.of(
            new Pose3d(15.079, 0.246, 1.356,new Rotation3d(0,0,Math.PI/2+0.523243287411175)),//1
            new Pose3d(16.209, 0.884, 1.356,new Rotation3d(0,0,Math.PI/2+0.523243287411175)),//2
            new Pose3d(16.579, 4.983, 1.451,new Rotation3d(0,0,Math.PI)),//3
            new Pose3d(16.579, 5.548, 1.451,new Rotation3d(0,0,Math.PI)),//4
            new Pose3d(14.700, 8.204, 0,new Rotation3d(0,0,-Math.PI/2)),//5
            new Pose3d(1.840, 8.204, 0,new Rotation3d(0,0,-Math.PI/2)),//6
            new Pose3d(-0.038, 5.548, 1.451,new Rotation3d(0,0,0)),//7
            new Pose3d(-0.038, 4.983, 1.451,new Rotation3d(0,0,0)),//8
            new Pose3d(0.356, 0.884, 1.356,new Rotation3d(0,0,Math.PI/2-0.523243287411175)),//9
            new Pose3d(1.461, 0.246, 1.356,new Rotation3d(0,0,Math.PI/2-0.523243287411175)),//10
            new Pose3d(11.900, 3.713, 1.322,new Rotation3d(0,0,-Math.PI*1/3)),//11
            new Pose3d(11.901, 4.498, 1.322,new Rotation3d(0,0,Math.PI*1/3)),//12
            new Pose3d(11.220, 4.106, 1.322,new Rotation3d(0,0,Math.PI)),//13
            new Pose3d(5.321, 4.105, 1.322,new Rotation3d(0,0,0)),//14
            new Pose3d(4.641, 4.498, 1.322,new Rotation3d(0,0,Math.PI*2/3)),//15
            new Pose3d(4.640, 3.714, 1.322,new Rotation3d(0,0,-Math.PI*2/3))//16
        );
        
        public static final Pose3d speakerPose3d=new Pose3d(0, 5.548, 2.05,new Rotation3d(0,0,0));//not precise         
    }


    public static final class LimelightConstant{
        public static final String tagLimelight = "limelight";
        public static final double apriltagHeight = 1.451;//Meters
        public static final double cameraHeight = 0.3302;//Meters  0.3302
        public static final double rotationPointHeight = 0.75;//Meters
        public static final double spearkerHeight = 1.978;//Meters
    
        public static final double rotationToCameraXdistance = 0.50;//Meters
    
        public static final double limelightOffsetAngle = 30.0;//Degress
        public static final double rotationPointOffsetAngle = 12.0;//Degress
    
        public static final double errorDistance_k = 1.124233;
        public static final double errorDistance_b = 0.122818;
    
        public static final double errorratation_k = 1.124233;
        public static final double errorratation_b = 0.122818;
      }

      public static final class LEDConstants{
        public static final int LEDPort = 9;
        public static final int length = 11;
    
        //solidColor
        public static final int initializelength = 0;
        public static final double TwoColorlength = 0.1;
    
        //strobe
        public static final double strobeDuration = 0.3;
        public static final double EstoppedstrobeDuration = 0.3;
        public static final double PickstrobeDuration = 0.25;
        public static final double CaststrobeDuration = 0.25;
        public static final double ScorestrobeDuration = 0.2;
    
        //rainbow
        public static final double ClimbrainbowcycleLength = 10.0;
        public static final double ClimbrainbowDurtion = 1.0;
    
        //breath
        public static final double breathDuration = 0.3;
        public static final double EnablebreathDuration = 0.6;
    
        //wave
        public static final double waveExponent = 0.4;
        public static final double DefaultwaveLength = 5;
        public static final double DefaultwaveDuration = 0.2;
    
        //stripes
        public static final int stripesLength = 3;
        public static final double stripesDuration = 4.0;
    
        public static final double autoFadeMaxTime = 5.0; // Return to normal
        public static final double autoFadeTime = 2.5; // 3s nominal
        public static final double autoFastCycleLength = 15;
        public static final double autoFastDuration = 0.25;
      
      }

}
