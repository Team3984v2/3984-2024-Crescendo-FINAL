package frc.robot.subsystems;

import java.io.UncheckedIOException;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;

import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro1;
  //ADXRS450_Gyro gyro;
  private SwerveDriveOdometry swerveOdometry;
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveModule[] mSwerveMods;
  private PhotonCamera cam;
  private Field2d field;
  private AprilTagFieldLayout layout;
  private SysIdRoutine m_steerRoutine;
  private SysIdRoutine m_driveRoutine;
  private final MutableMeasure<Voltage> m_appliedVoltage;
  private final MutableMeasure<Angle> m_angle;
  private final MutableMeasure<Velocity<Angle>> m_angularVelocity;
  private final MutableMeasure<Distance> m_distance;
  private final MutableMeasure<Velocity<Distance>> m_velocity;
  public Swerve(PhotonCamera cam) {
    this.cam = cam;
    gyro1 = new Pigeon2(5);
    gyro1.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());
    //poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), new Pose2d());
    try {
      layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (UncheckedIOException e) {
      System.out.println("April Tag Field Layout not Found");
    }
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getSpeeds, 
            this::setModuleStates, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
                new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
                Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
                Constants.Swerve.f1ModuleOffset.getNorm(), 
                new ReplanningConfig()
            ), 
            ()->{
                if (DriverStation.getAlliance().isPresent()){
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);
    m_appliedVoltage = mutable(Units.Volts.of(0));
    m_angle = mutable(Units.Rotations.of(0));
    m_angularVelocity = mutable(Units.RotationsPerSecond.of(0));
    m_distance = mutable(Units.Meters.of(0));
    m_velocity = mutable(Units.MetersPerSecond.of(0));
    Pose2d initPose = swerveOdometry.getPoseMeters();
    m_driveRoutine = new SysIdRoutine(
              new SysIdRoutine.Config(), 
              new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> voltageDrive(volts.in(Units.Volts)),
                log ->{
                  log.motor("drive-left")
                    .voltage(
                      m_appliedVoltage.mut_replace(
                        getSysIDSpeeds()[2] * RobotController.getBatteryVoltage(), Units.Volts))
                    .linearPosition(
                      m_distance.mut_replace(
                        Math.sqrt(Math.pow(swerveOdometry.getPoseMeters().getX() - initPose.getX(), 2) + Math.pow(swerveOdometry.getPoseMeters().getY() - initPose.getY(), 2)), Units.Meters))             
                    .linearVelocity(
                      m_velocity.mut_replace(
                        getDriveVelocities()[2], Units.MetersPerSecond));
                  log.motor("drive-right")
                    .voltage(
                      m_appliedVoltage.mut_replace(
                        getSysIDSpeeds()[3] * RobotController.getBatteryVoltage(), Units.Volts))
                    .linearPosition(
                      m_distance.mut_replace(
                        Math.sqrt(Math.pow(swerveOdometry.getPoseMeters().getX() - initPose.getX(), 2) + Math.pow(swerveOdometry.getPoseMeters().getY() - initPose.getY(), 2)), Units.Meters))             
                    .linearVelocity(
                      m_velocity.mut_replace(
                        getDriveVelocities()[3], Units.MetersPerSecond));
                }, this)
              );
    m_steerRoutine = new SysIdRoutine(
              new SysIdRoutine.Config(), 
              new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> voltageAngle(volts.in(Units.Volts)), 
                log->{
                  log.motor("front-right").voltage(
                    m_appliedVoltage.mut_replace(
                      getSysIDAngleSpeeds()[0] * RobotController.getBatteryVoltage(), Units.Volts)
                  ).angularPosition(
                    m_angle.mut_replace(
                      getStates()[0].angle.getRotations(), Units.Rotations)
                  ).angularVelocity(m_angularVelocity.mut_replace(
                    m_angularVelocity.mut_replace(
                      getAngleVelocities()[0], Units.RotationsPerSecond))
                  );
                  log.motor("front-left").voltage(
                    m_appliedVoltage.mut_replace(
                      getSysIDAngleSpeeds()[1] * RobotController.getBatteryVoltage(), Units.Volts)
                  ).angularPosition(
                    m_angle.mut_replace(
                      getStates()[1].angle.getRotations(), Units.Rotations)
                  ).angularVelocity(m_angularVelocity.mut_replace(
                    m_angularVelocity.mut_replace(
                      getAngleVelocities()[1], Units.RotationsPerSecond))
                  );
                  log.motor("back-left").voltage(
                    m_appliedVoltage.mut_replace(
                      getSysIDAngleSpeeds()[2] * RobotController.getBatteryVoltage(), Units.Volts)
                  ).angularPosition(
                    m_angle.mut_replace(
                      getStates()[2].angle.getRotations(), Units.Rotations)
                  ).angularVelocity(m_angularVelocity.mut_replace(
                    m_angularVelocity.mut_replace(
                      getAngleVelocities()[2], Units.RotationsPerSecond))
                  );
                  log.motor("back-right").voltage(
                    m_appliedVoltage.mut_replace(
                      getSysIDAngleSpeeds()[3] * RobotController.getBatteryVoltage(), Units.Volts)
                  ).angularPosition(
                    m_angle.mut_replace(
                      getStates()[3].angle.getRotations(), Units.Rotations)
                  ).angularVelocity(m_angularVelocity.mut_replace(
                    m_angularVelocity.mut_replace(
                      getAngleVelocities()[3], Units.RotationsPerSecond))
                  );
                }, this));
      
    
  }
  public Command angleSysIdDynamic(SysIdRoutine.Direction direction){
    return m_steerRoutine.dynamic(direction);
  }
  public Command angleSysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_steerRoutine.quasistatic(direction);
  }
  public Command driveSysIdDynamic(SysIdRoutine.Direction direction){
    return m_driveRoutine.dynamic(direction);
  }
  public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_driveRoutine.quasistatic(direction);
  }
  public void voltageAngle(double voltage){
    for (SwerveModule mod : mSwerveMods){
      mod.setAVoltage(voltage);
    }
  }
  public void voltageDrive(double voltage){
    for (SwerveModule mod : mSwerveMods){
      if (mod.moduleNumber == 0 ){
        voltage = voltage*-1;
      }
      mod.setDVoltage(-1*voltage);
    }
  }
  
  public Double[] getSysIDAngleSpeeds(){
    Double[] speeds = new Double[4];
    for (SwerveModule mod: mSwerveMods){
      speeds[mod.moduleNumber] = mod.getASpeed();
    }
    return speeds;
  }
  public Double[] getSysIDSpeeds(){
    Double[] speeds = new Double[4];
    for (SwerveModule mod: mSwerveMods){
      speeds[mod.moduleNumber] = mod.getDSpeed();
    }
    return speeds;
  }
  public Double[] getAngleVelocities(){
    Double[] velocities = new Double[4];
    for (SwerveModule mod: mSwerveMods){
      velocities[mod.moduleNumber] = mod.getAVelocity();
    }
    return velocities;
  }
  public Double[] getDriveVelocities(){
    Double[] velocities = new Double[4];
    for (SwerveModule mod: mSwerveMods){
      velocities[mod.moduleNumber] = mod.getDVelocity();
    }
    return velocities;
  }
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    System.out.println(chassisSpeeds.omegaRadiansPerSecond);
    SwerveModuleState[] desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond ));//new ChassisSpeeds(0,0,.5));// TODO
                                                                                                              // NEED TO
                                                                                                              // WORK
                                                                                                              // ONnnjnijni
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void XLock() {
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    desiredStates[0] = new SwerveModuleState(0, new Rotation2d(Mod0.angleOffset.getDegrees() + 45));
    desiredStates[1] = new SwerveModuleState(0, new Rotation2d(Mod1.angleOffset.getDegrees() - 45));
    desiredStates[2] = new SwerveModuleState(0, new Rotation2d(Mod2.angleOffset.getDegrees() - 45));
    desiredStates[3] = new SwerveModuleState(0, new Rotation2d(Mod3.angleOffset.getDegrees() + 45));
    setModuleStates(desiredStates);
  }
  public void resetWheels() {
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    desiredStates[0] = new SwerveModuleState(0, new Rotation2d(Mod0.angleOffset.getDegrees()));
    desiredStates[1] = new SwerveModuleState(0, new Rotation2d(Mod1.angleOffset.getDegrees()));
    desiredStates[2] = new SwerveModuleState(0, new Rotation2d(Mod2.angleOffset.getDegrees()));
    desiredStates[3] = new SwerveModuleState(0, new Rotation2d(Mod3.angleOffset.getDegrees()));
    setModuleStates(desiredStates);
  }
  public void stop() {
    setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
  }

  public void setAbsolute() {
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
    // Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }
  // Aims torwards a April Tag
  /*
   * public void turnToTarget(){
   * var lastestTarget = cam.getLatestResult();
   * if (lastestTarget.hasTargets()){
   * Pose3d tagPos =
   * layout.getTagPose(lastestTarget.getBestTarget().getFiducialId()).get();
   * Rotation2d target = PhotonUtils.getYawToPose(getPose(), tagPos.toPose2d());
   * 
   * //return runOnce()
   * }
   * else{
   * System.out.println("NO TARGETS FOUND");
   * }
   * PIDController thetaController = new
   * PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
   * thetaController.enableContinuousInput(-Math.PI, Math.PI);
   * Rotation2d targetYaw = PhotonUtils.getYawToPose(getPose(), getPose());
   * Translation2d toTarget = new Translation2d(0, targetYaw);
   * SwerveModuleState[] states =
   * Constants.Swerve.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.
   * fromFieldRelativeSpeeds(0,0, thetaController.calculate(previousTimeStamp),
   * getYaw()));
   * //Trajectory g = new TrajectoryGenerator(){}
   * }
   */

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
    //return poseEstimator.getEstimatedPosition();

  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    //poseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPoset();
    }
    return positions;
  }

  public ChassisSpeeds getSpeeds() {

    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void zeroGyro() {
    gyro1.reset();
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro1.getAngle() * (Constants.Swerve.invertGyro ? 1 : -1));

  }

  /* X out, Y to the left */
  /*
   * public Command moveToPose(Supplier<Pose2d> poseSupplier, Translation2d
   * Offset){
   * PIDController yPID = new PIDController(0, 0, 0);
   * PIDController xPID = new PIDController(0, 0, 0);
   * PIDController thetaPID = new PIDController(0, 0, 0);
   * thetaPID.enableContinuousInput(0, 2*Math.PI);
   * yPID.setTolerance(1);
   * xPID.setTolerance(1);
   * thetaPID.setTolerance(1);
   * return runOnce(()->{
   * Translation2d offset = Offset;
   * Pose2d pose = poseSupplier.get();
   * Rotation2d targetRot = pose.getRotation();
   * offset = Offset.rotateBy(targetRot);
   * Translation2d targetTranslation = pose.getTranslation();
   * Translation2d offseted = targetTranslation.plus(Offset);
   * field.getObject("CurrTarget").setPose(new Pose2d(offseted, targetRot));
   * 
   * xPID.setSetpoint(offseted.getX());
   * yPID.setSetpoint(offseted.getY());
   * 
   * thetaPID.setSetpoint(targetRot.minus(Rotation2d.fromDegrees(180)).getRadians(
   * ));
   * 
   * }).andThen(run(
   * () -> {
   * drive(new Translation2d(
   * xPID.calculate(swerveOdometryVision.getEstimatedPosition().getX()),
   * yPID.calculate(swerveOdometryVision.getEstimatedPosition().getY())),
   * thetaPID.calculate(swerveOdometryVision.getEstimatedPosition().getRotation().
   * getRadians()),
   * true,
   * false
   * );
   * }
   * )).until(()-> xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint()
   * ).andThen(()->{
   * xPID.close();
   * yPID.close();
   * thetaPID.close();
   * });
   * 
   * }
   */
  /*
   * public int getClosestTag(){
   * var lastest = cam.getLatestResult();
   * if (lastest.hasTargets()){
   * return lastest.getBestTarget().getFiducialId();
   * }
   * else{
   * return -1;
   * }
   * }
   * public Command moveToTag(int id, Translation2d offset){
   * return moveToPose(() -> layout.getTagPose(id).get().toPose2d(), offset);
   * }
   * public Command moveToTag(Translation2d offset){
   * var lastest = cam.getLatestResult();
   * if (lastest.hasTargets()){
   * return moveToPose(() ->
   * layout.getTagPose(lastest.getBestTarget().getFiducialId()).get().toPose2d(),
   * offset);
   * }
   * else{
   * return null;
   * }
   * }
   */

  /*
   * public Command alignWNearestRight(int id){
   * var lastest = cam.getLatestResult();
   * if (lastest.hasTargets()){
   * Pose3d tagPos =
   * layout.getTagPose(lastest.getBestTarget().getFiducialId()).get();
   * //Pose3d TransPos = new Pose3d(tagPos.getX() + 0.5, tagPos.getY(),
   * tagPos.getZ() + 1, tagPos.getRotation());
   * return moveToPose(()->tagPos.toPose2d(), Constants.AutoConstants.GOALRIGHT);
   * //TODO
   * }
   * 
   * return (new SequentialCommandGroup());
   * }
   * public Command alignWNearestLeft(int id){
   * var lastest = cam.getLatestResult();
   * if (lastest.hasTargets()){
   * Pose3d tagPos =
   * layout.getTagPose(lastest.getBestTarget().getFiducialId()).get();
   * //Pose3d TransPos = new Pose3d(tagPos.getX() + 0.5, tagPos.getY(),
   * tagPos.getZ() + 1, tagPos.getRotation());
   * return moveToPose(()->tagPos.toPose2d(), Constants.AutoConstants.GOALLEFT);
   * //TODO
   * }
   * 
   * return new SequentialCommandGroup();
   * }
   * public Command alignWNearestMiddle(int id){
   * var lastest = cam.getLatestResult();
   * if (lastest.hasTargets()){
   * Pose3d tagPos =
   * layout.getTagPose(lastest.getBestTarget().getFiducialId()).get();
   * //Pose3d TransPos = new Pose3d(tagPos.getX() + 0.5, tagPos.getY(),
   * tagPos.getZ() + 1, tagPos.getRotation());
   * return moveToPose(()->tagPos.toPose2d(), Constants.AutoConstants.GOALMIDDLE);
   * //TODO
   * }
   * 
   * return new SequentialCommandGroup();
   * }
   */

  @Override
  public void periodic() {

    swerveOdometry.update(getYaw(), getPositions());
    //poseEstimator.update(getYaw(), getPositions());
    //var l = cam.getLatestResult(); 
    /*if (l.hasTargets()){
      var imageCaptureTime = l.getTimestampSeconds();
      var camToTargetTrans = l.getBestTarget().getBestCameraToTarget();
      if (layout.getTagPose(l.getBestTarget().getFiducialId()).isPresent()){}*/
     // var camPose = layout.getTagPose(l.getBestTarget().getFiducialId()).get().transformBy(camToTargetTrans.inverse());
      //poseEstimator.addVisionMeasurement(
       //camPose.transformBy(Constants.Swerve.camera.CAMERA_TO_ROBOT).toPose2d(), imageCaptureTime);
    //}
    
    field.setRobotPose(getPose());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Mod" + mod.moduleNumber + " DrivePos", mod.getPosets());
    }
  }
}
