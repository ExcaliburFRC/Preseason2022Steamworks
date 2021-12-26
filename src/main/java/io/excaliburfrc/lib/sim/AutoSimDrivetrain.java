package io.excaliburfrc.lib.sim;

import static java.util.Objects.requireNonNull;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

import javax.print.DocFlavor.STRING;
import java.lang.annotation.Annotation;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.reflect.Field;
import java.util.List;

public class AutoSimDrivetrain {
  private final DifferentialDrivetrainSim m_model;
  private final MotorController m_leftMotor;
  private final MotorController m_rightMotor;
  private final SimEncoder m_leftEncoder;
  private final SimEncoder m_rightEncoder;
  private final SimGyro m_gyro;

  public AutoSimDrivetrain(
      DifferentialDrivetrainSim model,
      MotorController leftMotor,
      MotorController rightMotor,
      SimEncoder leftEncoder,
      SimEncoder rightEncoder,
      SimGyro gyro) {
    m_model = model;
    m_leftMotor = leftMotor;
    m_rightMotor = rightMotor;
    m_leftEncoder = leftEncoder;
    m_rightEncoder = rightEncoder;
    m_gyro = gyro;
  }

  @Retention(RetentionPolicy.RUNTIME)
  public @interface LeftEncoder {
    /** Only used for CAN encoders, don't use for DIO encoders. */
    int canID = -1;
  }

  @Retention(RetentionPolicy.RUNTIME)
  public @interface RightEncoder {
    /** Only used for CAN encoders, don't use for DIO encoders. */
    int canID = -1;
  }

  @Retention(RetentionPolicy.RUNTIME)
  public @interface LeftMotor {}

  @Retention(RetentionPolicy.RUNTIME)
  public @interface RightMotor {}

  @Retention(RetentionPolicy.RUNTIME)
  public @interface Gyro {}

  /** Beware!!! THIS WILL RETURN NULL ON A REAL ROBOT!!! */
  public static AutoSimDrivetrain Initialize(Object subsystem, DifferentialDrivetrainSim model) {
    if (RobotBase.isReal()) return null;
    Tracer tracer = new Tracer();

    DifferentialDrivetrainSim m_model = requireNonNull(model);
    MotorController m_leftMotor = null;
    MotorController m_rightMotor = null;
    SimEncoder m_leftEncoder = null;
    SimEncoder m_rightEncoder = null;
    SimGyro m_gyro = null;

    tracer.resetTimer();
    List<Field> fields = List.of(subsystem.getClass().getDeclaredFields());
    tracer.addEpoch("get fields");
    require(!fields.isEmpty(), "No fields found!");
    StringBuilder stringBuilder = new StringBuilder();
    try {
      for (int i = 0, fieldsSize = fields.size(); i < fieldsSize; i++) {
        stringBuilder.append(i);
        tracer.resetTimer();
        Field field = fields.get(i);
        // quick exit
        if (field.getAnnotations().length == 0) continue;
        field.setAccessible(true);
        tracer.addEpoch("set-accessible");
        {
          tracer.resetTimer();
          if (field.isAnnotationPresent(LeftMotor.class)) {
            requireSingleInitialization(m_leftMotor, LeftMotor.class);
            m_leftMotor = (MotorController) field.get(subsystem);
          }
          tracer.addEpoch("left motor");
          if (field.isAnnotationPresent(RightMotor.class)) {
            requireSingleInitialization(m_rightMotor, RightMotor.class);
            m_rightMotor = (MotorController) field.get(subsystem);
          }
          tracer.addEpoch("right motor");
        }
        {
          tracer.resetTimer();
          LeftEncoder lencoder;
          if ((lencoder = field.getAnnotation(LeftEncoder.class)) != null) {
            Class<?> encoderType = field.getType();
            if (RelativeEncoder.class.isAssignableFrom(encoderType)) {
              m_leftEncoder =
                      new SimREVEncoder(lencoder.canID, (RelativeEncoder) field.get(subsystem));
            } else if (Encoder.class.equals(encoderType)) {
              m_leftEncoder = new SimWPILibEncoder((Encoder) field.get(subsystem));
            } else {
              // fail.
              //noinspection ConstantConditions
              require(false, "Unsupported encoder type: " + encoderType.getSimpleName());
            }
          }
          tracer.addEpoch("left encoder" + i);
          System.out.println("left encoder");
          RightEncoder rencoder;
          if ((rencoder = field.getAnnotation(RightEncoder.class)) != null) {
            Class<?> encoderType = field.getType();
            if (RelativeEncoder.class.isAssignableFrom(encoderType)) {
              m_rightEncoder =
                      new SimREVEncoder(rencoder.canID, (RelativeEncoder) field.get(subsystem));
            } else if (Encoder.class.equals(encoderType)) {
              m_rightEncoder = new SimWPILibEncoder((Encoder) field.get(subsystem));
            } else {
              // fail.
              //noinspection ConstantConditions
              require(false, "Unsupported encoder type: " + encoderType.getSimpleName());
            }
          }
          tracer.addEpoch("right encoder" + i);
        }
        {
          tracer.resetTimer();
          if (field.isAnnotationPresent(Gyro.class)) {
            Class<?> gyroType;
            require(
                    (gyroType = field.getType()).equals(AHRS.class),
                    "Gyro is wrong type: " + gyroType.getSimpleName() + "; expected AHRS (NavX).");
            m_gyro = new SimNavx();
          }
          tracer.addEpoch("gyro" + i);
        }
        stringBuilder.append('\n');
        tracer.printEpochs(stringBuilder::append);
        tracer.clearEpochs();
      }
      requireInitialized(m_leftMotor, m_rightMotor, m_leftEncoder, m_rightEncoder, m_gyro);

    } catch (IllegalAccessException e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }
    System.err.println(stringBuilder);
    System.err.flush();
    return new AutoSimDrivetrain(
        m_model, m_leftMotor, m_rightMotor, m_leftEncoder, m_rightEncoder, m_gyro);
  }

  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_model.setInputs(
        m_leftMotor.get() * RobotController.getInputVoltage(),
        m_rightMotor.get() * RobotController.getInputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_model.update(0.02);

    // Update all of our sensors.
    m_leftEncoder.setPosition(m_model.getLeftPositionMeters());
    m_leftEncoder.setVelocity(m_model.getLeftVelocityMetersPerSecond());
    m_rightEncoder.setPosition(m_model.getRightPositionMeters());
    m_rightEncoder.setVelocity(m_model.getRightVelocityMetersPerSecond());
    m_gyro.setYaw(-m_model.getHeading().getDegrees());
  }

  private static void requireInitialized(Object... objs) {
    for (Object obj : objs) {
      require(obj != null, "Failed to initialize components!");
    }
  }

  private static void requireSingleInitialization(
      Object obj, Class<? extends Annotation> component) {
    require(obj == null, "Component" + component.getSimpleName() + "already initialized!");
  }

  private static void require(boolean condition, String message) {
    if (!condition) {
      throw new IllegalStateException(message);
    }
  }

  private abstract static class SimEncoder {
    abstract void setPosition(double position);

    abstract void setVelocity(double velocity);
  }

  private static final class SimWPILibEncoder extends SimEncoder {
    private final EncoderSim sim;

    SimWPILibEncoder(Encoder encoder) {
      sim = new EncoderSim(encoder);
    }

    @Override
    void setPosition(double position) {
      sim.setDistance(position);
    }

    @Override
    void setVelocity(double velocity) {
      sim.setRate(velocity);
    }
  }

  private static final class SimREVEncoder extends SimEncoder {
    private final SimDouble m_position;
    private final SimDouble m_velocity;

    SimREVEncoder(int canID, RelativeEncoder encoder) {
      boolean alt = encoder.getClass().equals(SparkMaxAlternateEncoder.class);
      SimDeviceSim device = new SimDeviceSim("SPARK MAX ", canID);
      m_position = device.getDouble(alt ? "Alt Encoder Position" : "Position");
      m_velocity = device.getDouble(alt ? "Alt Encoder Velocity" : "Velocity");
    }

    @Override
    void setPosition(double position) {
      m_position.set(position);
    }

    @Override
    void setVelocity(double velocity) {
      m_velocity.set(velocity);
    }
  }

  private abstract static class SimGyro {
    abstract void setYaw(double degrees);
  }

  private static final class SimNavx extends SimGyro {
    SimDouble m_yawDegrees;

    SimNavx() {
      m_yawDegrees = new SimDeviceSim("navX-Sensor", 0).getDouble("Yaw");
    }

    @Override
    void setYaw(double degrees) {
      m_yawDegrees.set(degrees);
    }
  }
}
