package frc.team2767.visiontf;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team2767.visiontf.control.Controls;
import frc.team2767.visiontf.subsystem.DriveSubsystem;
import io.reactivex.disposables.Disposable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.deadeye.DeadeyeService;
import org.strykeforce.thirdcoast.telemetry.TelemetryController;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;

import java.util.Date;

public class Robot extends TimedRobot {
  public static final TelemetryService TELEMETRY = new TelemetryService(TelemetryController::new);
  public static final DeadeyeService DEADEYE_SERVICE = new DeadeyeService();
  public static final DriveSubsystem DRIVE = new DriveSubsystem();
  // Controls initialize Commands so this should be instantiated last to prevent
  // NullPointerExceptions in commands that require() Subsystems above.
  public static final Controls CONTROLS = new Controls();
  public static Disposable disposable;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  @Override
  public void robotInit() {
    logger.info("Today is {}", new Date());
    TELEMETRY.start();
    DEADEYE_SERVICE.enableConnectionEventLogging(true);
    //    DEADEYE_SERVICE
    //        .getMessageObservable()
    //        .subscribe(deadeyeMessage -> logger.info(deadeyeMessage.toString()));
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }
}
