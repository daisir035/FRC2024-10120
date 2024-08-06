package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private Field2d m_field = new Field2d();
    private Pose2d m_Pose2d = new Pose2d();

    public Translation2d currentDesireSpeed =  new Translation2d(0,0);
    public double currentDesireAngularSpeed = 0;

    Constraints  i = new Constraints(10,1);
    ProfiledPIDController calyaw = new ProfiledPIDController(2,0,0.3,i);

    LimeLightSubsystem m_LightSubsystem;

    public enum DriveState{
        AMP,
        PASSING,
        SPEAKER,
        TELEOP,
        STOP,
        SOURCE
    }
    public Pose2d assistant = new Pose2d(0,0,new Rotation2d(0) );
    
    public DriveState currentDriveState = DriveState.TELEOP;
    public void update(DriveState driveState){
        this.currentDriveState = driveState;
    }

    public DriveSubsystem(LimeLightSubsystem lightSubsystem) {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        this.m_LightSubsystem = lightSubsystem;
        calyaw.enableContinuousInput(Math.PI, -Math.PI);

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        translation=optimizeDesireChassisVelocity(translation,currentDesireSpeed);
        currentDesireSpeed=translation;
        rotation=optimizeDesireChassisAngularSpeed(rotation,currentDesireAngularSpeed);
        currentDesireAngularSpeed=rotation;

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxModuleSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        SmartDashboard.putData("Field",m_field);//hasn't been tested
            new Thread(()->{
        }).start();

    }    

    public Translation2d optimizeDesireChassisVelocity(Translation2d desireChassisVelocity,Translation2d currentDesireChassisVelocity){
        Translation2d currentChassisSpeed=currentDesireChassisVelocity;
        Rotation2d headingAngle=currentChassisSpeed.getAngle();
        Rotation2d desireDirection=desireChassisVelocity.getAngle();
        double rotatedCurrentSpeed=currentChassisSpeed.rotateBy(desireDirection.times(-1)).getX();
        double desireChassisSpeed=desireChassisVelocity.getNorm();
    
        Translation2d deltaV=desireChassisVelocity.minus(currentChassisSpeed);
        Translation2d acceleration=new Translation2d(Constants.Swerve.maxCentripetalAcceleration,0).rotateBy(deltaV.getAngle());
        if (rotatedCurrentSpeed<0){
        desireChassisSpeed=0;
        }else if(rotatedCurrentSpeed<desireChassisSpeed){
          desireChassisSpeed=rotatedCurrentSpeed;
        }
        desireChassisVelocity=new Translation2d(desireChassisSpeed,0).rotateBy(desireDirection); 
        Translation2d rotatedDesireAcceleration=acceleration.rotateBy(headingAngle.times(-1));
        if((Constants.Swerve.maxAcceleration-rotatedDesireAcceleration.getX())<0||currentChassisSpeed.getNorm()==0){
          acceleration=new Translation2d(Constants.Swerve.maxAcceleration,0).rotateBy(acceleration.getAngle());
        }
        if((Constants.Swerve.maxDeceleration+rotatedDesireAcceleration.getX())<0){
          acceleration=new Translation2d(Constants.Swerve.maxDeceleration,0).rotateBy(acceleration.getAngle());
        }
        Translation2d changeSpeed=acceleration.times(0.02);
          if(changeSpeed.getNorm()<deltaV.getNorm()){
            desireChassisVelocity=currentChassisSpeed.plus(changeSpeed);
        }
        return desireChassisVelocity;
      }

    public double optimizeDesireChassisAngularSpeed(double desireAngularSpeed,double currentDesireChassisAngularSpeed){
        double currentAngularSpeed = currentDesireChassisAngularSpeed;
        double angularAcceleration;
        double deltaOmega=desireAngularSpeed-currentAngularSpeed;
        if (Math.abs(desireAngularSpeed)>Math.abs(currentAngularSpeed)){
          angularAcceleration=Constants.Swerve.maxAngularAcceleration;
        }else{
          angularAcceleration=Constants.Swerve.maxAngularDeceleration;
        }
        double changeSpeed=angularAcceleration*0.02;
        if(deltaOmega<0){
          changeSpeed=changeSpeed*-1;
        }
        if(Math.abs(changeSpeed)<Math.abs(deltaOmega)){
          desireAngularSpeed=currentAngularSpeed+changeSpeed;
        }
        return desireAngularSpeed;
    }



    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

      public Rotation2d getDriverHeading(){
        Rotation2d heading=getHeading();
        if(getAlliance()==DriverStation.Alliance.Red){
            heading=heading.plus(new Rotation2d(Math.PI));
        }
        return heading;
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

   public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), 
        inversePose2dUsingAlliance(new Pose2d(0,0,new Rotation2d(0)),DriverStation.Alliance.Blue).getRotation()));
        gyro.setYaw(inversePose2dUsingAlliance(new Pose2d(0,0,new Rotation2d(0)),DriverStation.Alliance.Blue).getRotation().getDegrees());
    }


    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }


    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        switch (currentDriveState) {
            case TELEOP->{
                assistant=new Pose2d(0,0,new Rotation2d(0) );
            }
            case SPEAKER->{
                assistant=new Pose2d(0,0,new Rotation2d(calyaw.calculate(m_LightSubsystem.getCamearXdegree()*Math.PI/180*0.9,0)));
            }
            case AMP->{
                assistant=new Pose2d(0,0,new Rotation2d(-calyaw.calculate(-Math.PI/2-getGyroYaw().getRadians(),0)));
            }
            case PASSING->{
                assistant=new Pose2d(0,0,new Rotation2d(-calyaw.calculate(
                    inversePose2dUsingAlliance(new Pose2d(0,0, new Rotation2d((147.36-180)*Math.PI/180)),DriverStation.Alliance.Blue).getRotation().getRadians()
                    -getGyroYaw().getRadians(),0)));
            }
            case SOURCE->{
                assistant=assistant=new Pose2d(0,0,new Rotation2d(-calyaw.calculate(
                    inversePose2dUsingAlliance(new Pose2d(0,0, new Rotation2d((-56.73)*Math.PI/180)),DriverStation.Alliance.Blue).getRotation().getRadians()
                    -getGyroYaw().getRadians(),0)));
            }
            default->{
                assistant=new Pose2d(0,0,new Rotation2d(0) );
            }
                
        }
        SmartDashboard.putNumber("gyro", gyro.getAngle());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        
        m_field.setRobotPose(
            new Pose2d(swerveOdometry.getPoseMeters().getTranslation().plus(
            new Translation2d(0,0)),swerveOdometry.getPoseMeters().getRotation()));
    }

    //auto***************************************************************************************************

    public PathPlannerPath generatePath(String pathName){
        return PathPlannerPath.fromPathFile(pathName);
    }

    //return an auto command
    public Command followPathCommand(PathPlannerPath path){
        return 
        new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(Constants.AutoConstants.kPTranslationController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(Constants.AutoConstants.kPRotationController, 0.0, 0.0), // Rotation PID constants
                        Constants.Swerve.maxModuleSpeed, // Max module speed, in m/s
                        Math.sqrt(2)*Constants.Swerve.wheelBase/2, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }
    
    //return the chassis speed for the followpathholonomic method
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    //ChassisSpeeds supplier for the followpathholonomic method
    public void setChassisSpeeds (ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds); //Generate the swerve module states
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxModuleSpeed);
        setModuleStates(moduleStates);
    }

    public Pose2d inversePose2dUsingAlliance(Pose2d pose,DriverStation.Alliance allianceColor){
        if(allianceColor!=getAlliance()){
            return new Pose2d(16.54-pose.getX(),pose.getY(),new Rotation2d(Math.PI).minus(pose.getRotation()));
        }
        return pose;
    }

    public DriverStation.Alliance getAlliance(){
        try{
            return DriverStation.getAlliance().get();
        }catch(Exception e){
            return DriverStation.Alliance.Red;
        }
    }

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                    for(SwerveModule mod : mSwerveMods){
                        mod.getmDriveMotor().setVoltage(volts.in(Volts));
                    }
                },
                log -> {
                    for(SwerveModule mod : mSwerveMods){
                        log.motor(mod.getmDriveMotor().getDescription())
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                    mod.getmDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                            .linearPosition(m_distance.mut_replace(swerveOdometry.getPoseMeters().getTranslation().getDistance(new Translation2d(0,0)), Meters))
                            .linearVelocity(
                                m_velocity.mut_replace(mod.getState().speedMetersPerSecond, MetersPerSecond));
                    }       
                },
                this));
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

   
}