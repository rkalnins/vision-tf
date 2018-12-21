package frc.team2767.visiontf.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team2767.visiontf.Robot;
import frc.team2767.visiontf.command.TeleOpDriveCommand;
import io.reactivex.schedulers.Schedulers;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.deadeye.DeadeyeMessage;
import org.strykeforce.thirdcoast.deadeye.DeadeyeService;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.Wheel;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;
import org.strykeforce.thirdcoast.telemetry.grapher.Measure;
import org.strykeforce.thirdcoast.telemetry.item.Item;

import java.util.Collections;
import java.util.EnumSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends Subsystem implements Item {

  private static final double DRIVE_SETPOINT_MAX = 0.0;
  private static final double ROBOT_LENGTH = 1.0;
  private static final double ROBOT_WIDTH = 1.0;
  private static final double kP = 0.0022;
  private static final DeadeyeService DEADEYE = Robot.DEADEYE_SERVICE;
  private static DeadeyeMessage deadeyeMessage;
  private static double graphableYaw;
  private static double graphableError;
  private final SwerveDrive swerve = getSwerve();
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public DriveSubsystem() {
    Robot.TELEMETRY.register(this);
    deadeyeMessage = new DeadeyeMessage(new byte[] {});
    DEADEYE
        .getMessageObservable()
        .filter(m -> m.type == DeadeyeMessage.Type.DATA)
        .subscribeOn(Schedulers.io())
        .subscribe(msg -> deadeyeMessage = msg);

    logger.info("drive subsystem initialized");
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new TeleOpDriveCommand());
  }

  @Override
  public void periodic() {
    //    logger.debug("{}", getYaw());
  }

  public void setDriveMode(DriveMode mode) {
    logger.debug("setting drive mode to {}", mode);
    swerve.setDriveMode(mode);
  }

  public void drive(double forward, double strafe, double azimuth) {
    azimuth = getYaw();

    if (forward < 0.01 && forward > -0.01 && strafe > -0.01 && strafe < 0.01) {
      azimuth = 0.0;
    }
    logger.debug("{}", azimuth);

    swerve.drive(forward, strafe, azimuth);
  }

  public double getYaw() {
    double yaw = kP * getYawError();

    double maxV = 0.5;

    if (yaw > maxV) {
      yaw = maxV;
    } else if (yaw < -maxV) {
      yaw = -maxV;
    }

    graphableYaw = yaw;

    return yaw;
  }

  private double getYawError() {
    DeadeyeMessage message = deadeyeMessage;

    if (message == null) {
      return 0.0;
    }

    double error = (message.data[0] + (message.data[3] / 2)) - 320;

    graphableError = error;
    return error;
  }

  // Swerve configuration

  private SwerveDrive getSwerve() {
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.wheels = getWheels();
    config.gyro = new AHRS(SPI.Port.kMXP);
    config.length = ROBOT_LENGTH;
    config.width = ROBOT_WIDTH;
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    return new SwerveDrive(config);
  }

  private Wheel[] getWheels() {
    TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
    azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;
    azimuthConfig.slot_0.kP = 10.0;
    azimuthConfig.slot_0.kI = 0.0;
    azimuthConfig.slot_0.kD = 100.0;
    azimuthConfig.slot_0.kF = 0.0;
    azimuthConfig.slot_0.integralZone = 0;
    azimuthConfig.slot_0.allowableClosedloopError = 0;
    azimuthConfig.motionAcceleration = 10_000;
    azimuthConfig.motionCruiseVelocity = 800;

    TalonSRXConfiguration driveConfig = new TalonSRXConfiguration();
    driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    driveConfig.continuousCurrentLimit = 40;
    driveConfig.peakCurrentDuration = 0;
    driveConfig.peakCurrentLimit = 0;

    TelemetryService telemetryService = Robot.TELEMETRY;
    telemetryService.stop();

    Wheel[] wheels = new Wheel[4];

    for (int i = 0; i < 4; i++) {
      TalonSRX azimuthTalon = new TalonSRX(i);
      azimuthTalon.configAllSettings(azimuthConfig);

      TalonSRX driveTalon = new TalonSRX(i + 10);
      driveTalon.configAllSettings(driveConfig);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      telemetryService.register(azimuthTalon);
      telemetryService.register(driveTalon);

      Wheel wheel = new Wheel(azimuthTalon, driveTalon, DRIVE_SETPOINT_MAX);
      wheels[i] = wheel;
    }

    return wheels;
  }

  @NotNull
  @Override
  public String getDescription() {
    return "Drive Subsystem";
  }

  @NotNull
  @Override
  public String getType() {
    return this.getClass().getName();
  }

  @Override
  public int getDeviceId() {
    return 0;
  }

  @NotNull
  @Override
  public Set<Measure> getMeasures() {
    Set<Measure> measures = EnumSet.of(Measure.COMPONENT_YAW, Measure.CLOSED_LOOP_ERROR);
    return Collections.unmodifiableSet(measures);
  }

  @NotNull
  @Override
  public DoubleSupplier measurementFor(@NotNull Measure measure) {
    switch (measure) {
      case COMPONENT_YAW:
        return () -> graphableYaw;
      case CLOSED_LOOP_ERROR:
        return () -> -graphableError;
      default:
        throw new IllegalStateException(measure.toString());
    }
  }

  @Override
  public int compareTo(@NotNull Item other) {
    int result = getDescription().compareTo(other.getDescription());
    if (result != 0) return result;
    else return Integer.compare(getDeviceId(), other.getDeviceId());
  }
}
