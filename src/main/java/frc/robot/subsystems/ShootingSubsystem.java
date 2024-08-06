package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanID;
import frc.robot.Constants.IntakeConstant;
import frc.robot.Constants.ShootingConstant;
import frc.robot.subsystems.FeedingSubsystem.FeedingState;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class ShootingSubsystem extends SubsystemBase {
  private TalonFX shootingUpMotor = new TalonFX(CanID.UpShootingID);
  private TalonFX shootingDownMotor = new TalonFX(CanID.DownShootingID);

  private TalonFXConfiguration flyWheelUpconfig = new TalonFXConfiguration();
  private TalonFXConfiguration flyWheelDownconfig = new TalonFXConfiguration();

  private VelocityVoltage shootingVeltageUp = new VelocityVoltage(0);
  private VelocityVoltage shootingVeltageDown = new VelocityVoltage(0);
    

  FeedingSubsystem m_FeedingSubsystem;
  LimeLightSubsystem m_LimeLightSubsystem;
  LEDSubsystem m_LedSubsystem;

  private boolean autoAimingActive = false;
  public ShooterSpeed speed;

  public class ShooterSpeed {
    double topMotorSpeed;
    double bottomMotorSpeed;

    public ShooterSpeed(double top, double bottom) {
      topMotorSpeed = top;
      bottomMotorSpeed = bottom;
    }
  }

  public class ShooterCalibration {
    double distance;
    ShooterSpeed speed;

    public ShooterCalibration(double distance, ShooterSpeed speed) {
      this.distance = distance;
      this.speed = speed;
    }
  }

  public ShooterCalibration[] shooterCalibration = {
    new ShooterCalibration(0.91, new ShooterSpeed(20.0, 53.33)),
    new ShooterCalibration(1.19, new ShooterSpeed(25.0, 41.66)),
    new ShooterCalibration(1.51, new ShooterSpeed(50.0, 50.0)),
    new ShooterCalibration(1.60, new ShooterSpeed(60.0, 41.0)),
    new ShooterCalibration(1.70, new ShooterSpeed(70.0, 32.0)),
    new ShooterCalibration(1.75, new ShooterSpeed(80.5, 35.0)),
    new ShooterCalibration(1.80, new ShooterSpeed(78.0, 33.0)),
    new ShooterCalibration(1.85, new ShooterSpeed(75.0, 33.0)),
    new ShooterCalibration(2.00, new ShooterSpeed(75.0, 30.0)),
    new ShooterCalibration(2.5, new ShooterSpeed(70, 25)),
    new ShooterCalibration(2.75, new ShooterSpeed(65, 25)),
    new ShooterCalibration(3, new ShooterSpeed(50, 25)),
    new ShooterCalibration(5, new ShooterSpeed(40, 15)),
  };


  public ShootingSubsystem(FeedingSubsystem feedingSubsystem,LimeLightSubsystem limeLightSubsystem,LEDSubsystem ledSubsystem) {
    this.m_FeedingSubsystem = feedingSubsystem;
    this.m_LimeLightSubsystem = limeLightSubsystem;
    this.m_LedSubsystem = ledSubsystem;

    flyWheelUpconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flyWheelUpconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flyWheelUpconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    flyWheelUpconfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    flyWheelUpconfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    flyWheelUpconfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    flyWheelUpconfig.CurrentLimits.SupplyTimeThreshold = 0.02;

    flyWheelUpconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
    flyWheelUpconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    flyWheelUpconfig.Slot0.kP = ShootingConstant.KP;
    flyWheelUpconfig.Slot0.kI = ShootingConstant.KI;
    flyWheelUpconfig.Slot0.kD = ShootingConstant.KD;
    flyWheelUpconfig.Slot0.kS = ShootingConstant.KS;
    flyWheelUpconfig.Slot0.kV = ShootingConstant.KV;
    flyWheelUpconfig.Slot0.kA = ShootingConstant.KA;

    flyWheelDownconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flyWheelDownconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flyWheelDownconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;
    flyWheelDownconfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    flyWheelDownconfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    flyWheelDownconfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    flyWheelDownconfig.CurrentLimits.SupplyTimeThreshold = 0.02;
    flyWheelDownconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
    flyWheelDownconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;
    flyWheelDownconfig.Slot0.kP = ShootingConstant.KP;
    flyWheelDownconfig.Slot0.kI = ShootingConstant.KI;
    flyWheelDownconfig.Slot0.kD = ShootingConstant.KD;
    flyWheelDownconfig.Slot0.kS = ShootingConstant.KS;
    flyWheelDownconfig.Slot0.kV = ShootingConstant.KV;
    flyWheelDownconfig.Slot0.kA = ShootingConstant.KA;

    shootingUpMotor.getConfigurator().apply(flyWheelUpconfig);
    shootingDownMotor.getConfigurator().apply(flyWheelDownconfig);
  }

  public enum ShootingState{
    Stop,
    Lowposition,
    Hightposition,
    AutoShooting,
    Amp,
    Pass,
    Auto,
    Default,
    Preparing
  }

  public ShootingState shootingState = ShootingState.Stop;

  public void update(){
    this.update(this.shootingState);
  }

  public void update(ShootingState state){
    this.shootingState = state;
    switch (state) {
      case Stop:
        stopMotor();
        this.m_FeedingSubsystem.stopMotor();
        break;

      case Lowposition:
        setVelocity(ShootingConstant.lowPositionVel_Up,ShootingConstant.lowPositionVel_Down);
        if (getDownErrorVel(ShootingConstant.lowPositionVel_Down)<2.0) {
          this.m_FeedingSubsystem.update(FeedingState.Shooting);
        }else{
          this.m_FeedingSubsystem.update(FeedingState.Stop);
        }
        break;

      case Hightposition:
        setVelocity(ShootingConstant.hightPositionVel_Up,ShootingConstant.hightPositionVel_Down);
        if (getUpErrorVel(ShootingConstant.hightPositionVel_Down)<3.0) {
          this.m_FeedingSubsystem.update(FeedingState.Shooting);//
        }else{
          this.m_FeedingSubsystem.update(FeedingState.Stop);
        }
        break;

      case AutoShooting:
      setCurrentSpeed();
      break;

      case Amp:
        setVelocity(ShootingConstant.armPOsitionVel_Up,ShootingConstant.armPOsitionVel_Down);
        if (getDownErrorVel(ShootingConstant.armPOsitionVel_Down)<1.0) {
          this.m_FeedingSubsystem.update(FeedingState.Shooting);
        }else{
          this.m_FeedingSubsystem.update(FeedingState.Stop);
      }
        break;

      case Pass:
        setVelocity(ShootingConstant.passPOsitionVel_Up,ShootingConstant.passPOsitionVel_Down);
        if (getUpErrorVel(ShootingConstant.passPOsitionVel_Up)<5.0) {
        this.m_FeedingSubsystem.update(FeedingState.Shooting);
        }else{
        this.m_FeedingSubsystem.update(FeedingState.Stop);
      }
      break;

      case Auto:
        setAutonomousCurrentSpeed();
        this.m_FeedingSubsystem.update(FeedingState.Auto);
        break;

      case Preparing:
        setAutonomousCurrentSpeed();
        this.m_FeedingSubsystem.update(FeedingState.Stop);
        break;

      case Default:
        setVelocity(ShootingConstant.defaultVel_Up,ShootingConstant.defaultVel_Down);
        this.m_FeedingSubsystem.update(FeedingState.Stop);
        break;
      

      default:
        break;
    }
  }

  private ShooterSpeed speedFromDistance(double meters) {
    double distance = meters;
    ShooterCalibration priorEntry = null;
    ShooterSpeed speed = null;

    for (ShooterCalibration calibration : shooterCalibration) {
      if (distance <= calibration.distance) {
        if (priorEntry == null) {
          // Anything closer that minimum calibration distance gets the same speed as minimum distance
          speed = calibration.speed;
        } else {
          // Linear interpolation between calibration entries
          double fraction = (distance - priorEntry.distance) / (calibration.distance - priorEntry.distance);

          speed = new ShooterSpeed(
            fraction * calibration.speed.topMotorSpeed + (1 - fraction) * priorEntry.speed.topMotorSpeed,
            fraction * calibration.speed.bottomMotorSpeed + (1 - fraction) * priorEntry.speed.bottomMotorSpeed
          );
        }

        break;
      }
      priorEntry = calibration;
    }

    // NOTE: Might be null if the calibration distance has been exceeded
    if (speed == null) {
      return speed = new ShooterSpeed(75.0, 30.0);
    }
    return speed;
  }

  private boolean isAuto() {
    if(this.m_LimeLightSubsystem.getCamearXDistance()>0.9&&
    this.m_LimeLightSubsystem.getCamearXDistance()<3.0&&
    Math.abs(this.m_LimeLightSubsystem.getrawwCamearYdegree())!=0){
      autoAimingActive = true;
    }else{
      autoAimingActive = false;
    }
    return autoAimingActive;
  }


  private void setCurrentSpeed() {
    double distance = this.m_LimeLightSubsystem.getCamearXDistance();
    speed = speedFromDistance(distance);
    shootingVeltageUp.Velocity = speed.topMotorSpeed;
    shootingVeltageDown.Velocity = speed.bottomMotorSpeed;
    shootingUpMotor.setControl(shootingVeltageUp);
    shootingDownMotor.setControl(shootingVeltageDown);
    // SmartDashboard.putBoolean("ready", getUpErrorVel(speed.bottomMotorSpeed)<3.0&&getDownErrorVel(speed.topMotorSpeed)<3.0);
    if (getUpErrorVel(speed.topMotorSpeed)<2.0&&getDownErrorVel(speed.bottomMotorSpeed)<2.0) {
      this.m_FeedingSubsystem.update(FeedingState.Auto);
    }else{
      this.m_FeedingSubsystem.update(FeedingState.Stop);
    }
    SmartDashboard.putNumber("topMotorSpeed", speed.topMotorSpeed);
    SmartDashboard.putNumber("UpMotorSpeed", speed.bottomMotorSpeed);
  }
  private void setAutonomousCurrentSpeed() {
    double distance = this.m_LimeLightSubsystem.getCamearXDistance();
    // double distance = m_DriveSubsystem.getPose().getTranslation().getDistance(new Translation2d(16.579, 5.548));
    SmartDashboard.putNumber("distance", distance);
    speed = speedFromDistance(distance);
    shootingVeltageUp.Velocity = speed.topMotorSpeed;
    shootingVeltageDown.Velocity = speed.bottomMotorSpeed;
    shootingUpMotor.setControl(shootingVeltageUp);
    shootingDownMotor.setControl(shootingVeltageDown);
    // SmartDashboard.putBoolean("ready", getUpErrorVel(speed.bottomMotorSpeed)<3.0&&getDownErrorVel(speed.topMotorSpeed)<3.0);
    
    // SmartDashboard.putNumber("topMotorSpeed", speed.topMotorSpeed);
    // SmartDashboard.putNumber("UpMotorSpeed", speed.bottomMotorSpeed);
  }
 
  public void setVelocity(double upvelocity,double downvelocity){
    shootingVeltageUp.Velocity = upvelocity;
    shootingVeltageDown.Velocity = downvelocity;
    shootingUpMotor.setControl(shootingVeltageUp);
    shootingDownMotor.setControl(shootingVeltageDown);
  }

  public void stopMotor(){
    shootingUpMotor.stopMotor();
    shootingDownMotor.stopMotor();
  }

  public double getdownVelocity(){
    return shootingDownMotor.getVelocity().getValue();
  }
  public double getupVelocity(){
    return shootingUpMotor.getVelocity().getValue();
  }

  public double getDownErrorVel(double target){
    return Math.abs(getdownVelocity()-target);
  }

  public double getUpErrorVel(double target){
    return Math.abs(getupVelocity()-target);
  }

  public boolean isAutoAimingActive() {
    return autoAimingActive;
  }

  @Override
  public void periodic() {
    if(this.m_FeedingSubsystem.getDownOptoeSwitch()){
      this.m_LedSubsystem.update(LEDState.North);
      if (this.m_FeedingSubsystem.getDownOptoeSwitch()&&isAutoAimingActive()) {
        this.m_LedSubsystem.update(LEDState.SCORETING);
      }
    }else{
      this.m_LedSubsystem.update(LEDState.INITIAL);
    }
    
    SmartDashboard.putNumber("ShootingVelocityUp", getupVelocity());
    SmartDashboard.putNumber("ShootingVelocityDown", getdownVelocity());
    // SmartDashboard.putNumber("distance", m_LimeLightSubsystem.getCamearXDistance());
    SmartDashboard.putBoolean("isAutoAimingActive", isAuto());
  }
}
