
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanID;
import frc.robot.Constants.ElevatorConstant;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import pabeles.concurrency.IntOperatorTask.Max;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor = new TalonFX(CanID.elevatorID);

  private TalonFXConfiguration elevatorconfig = new TalonFXConfiguration();
  private MotionMagicVoltage elevatorMagic = new MotionMagicVoltage(
    0, false, 0.1, 0, 
    false, false, false);


  private Servo m_Servo = new Servo(CanID.servoID);
  private double servoAngle = 0.0;


  private Timer m_Timer = new Timer();
  LEDSubsystem m_LedSubsystem;

  public ElevatorSubsystem(LEDSubsystem ledSubsystem) {
    this.m_LedSubsystem = ledSubsystem;

    elevatorconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    elevatorconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorconfig.CurrentLimits.SupplyCurrentLimit = 40;
    elevatorconfig.CurrentLimits.SupplyCurrentThreshold = 60;
    elevatorconfig.CurrentLimits.SupplyTimeThreshold = 0.01;

     /* Open and Closed Loop Ramping */
    elevatorconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
    elevatorconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    elevatorconfig.Slot0.kP = ElevatorConstant.KP;
    elevatorconfig.Slot0.kI = ElevatorConstant.KI;
    elevatorconfig.Slot0.kD = ElevatorConstant.KD;
    elevatorconfig.Slot0.kV = ElevatorConstant.KV;
    elevatorconfig.Slot0.kS = ElevatorConstant.KS;
    elevatorconfig.Slot0.kA = ElevatorConstant.KA;
    elevatorconfig.MotionMagic.MotionMagicAcceleration = ElevatorConstant.Acc;
    elevatorconfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstant.Vel;
    elevatorconfig.MotionMagic.MotionMagicJerk = ElevatorConstant.Jer;

    elevatorMotor.getConfigurator().apply(elevatorconfig);

    zeroPosition();
  }

  public enum ElevatorState{
    Stop,
    Up,
    Release,
    TestUp,
    TestRelease,
    Lock,
    LockRelease
  }

  public ElevatorState elevatorState = ElevatorState.Stop;

  public void update(){
    this.update(this.elevatorState);
  }

  public void update(ElevatorState state){
    this.elevatorState = state;
    switch (state) {
      case Stop:
        stopMotor();
        break;

      case Up:
        setPosition(ElevatorConstant.height_Up);
        if (Math.abs(getPosition()-ElevatorConstant.height_Up)<=10.0) {
          setservoAngle(ElevatorConstant.upAngle);
        }
        this.m_LedSubsystem.update(LEDState.CLIMBUp);
        break;

      case Release:
        setservoAngle(ElevatorConstant.releaseAngle);
        if (Math.abs(getServoAngle()-ElevatorConstant.releaseAngle)<=0.05) {
          setPosition(ElevatorConstant.height_Release);
        }
         this.m_LedSubsystem.update(LEDState.CLIMBRealse);
        break;

      case TestUp:
        setPercentOutput(ElevatorConstant.TestUPSpeed);
        break;

      case TestRelease:
        setservoAngle(ElevatorConstant.releaseAngle);
        if (Math.abs(getServoAngle()-ElevatorConstant.releaseAngle)<=0.05) {
        setPercentOutput(ElevatorConstant.TestReleaseSpeed);
        }
        break;

      case LockRelease:
        m_Servo.set(ElevatorConstant.releaseAngle);
      break;

      case Lock:
        m_Servo.set(ElevatorConstant.upAngle);
      break;

      default:
        break;
    }
  }

  public void setPosition(double height){
    height = MathUtil.clamp(height, ElevatorConstant.MinHeight,ElevatorConstant.MaxHeight);
    elevatorMagic.Position = height;
    elevatorMotor.setControl(elevatorMagic);
  }

  public void setPercentOutput(double velocity){
    elevatorMotor.set(velocity);
  }

  public void setservoAngle(double angle){
    m_Servo.set(angle);
  }

  public void zeroPosition(){
    elevatorMotor.setPosition(0);
    m_Servo.set(0.0);
  }

  public void stopMotor(){
    elevatorMotor.stopMotor();
  }

  public double getPosition(){
    return elevatorMotor.getPosition().getValue();
  }

  public double getCurrent(){
    return elevatorMotor.getSupplyCurrent().getValue();
  }

  public double getVelocity(){
    return elevatorMotor.getVelocity().getValue();
  }

  public double getServoAngle(){
    return m_Servo.get();
  }


  @Override
  public void periodic() {
    // SmartDashboard.putNumber("ElevatorPosition", getPosition());
    // SmartDashboard.putNumber("ElevatorCurrent", getCurrent());
    // SmartDashboard.putNumber("ElevatorVelocity", getVelocity());
    // SmartDashboard.putNumber("ServoAngle", getServoAngle());
  }
}
