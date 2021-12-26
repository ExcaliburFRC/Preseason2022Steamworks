package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.DriveConstants.GEARING;
import static io.excaliburfrc.robot.Constants.DriveConstants.TRACK_WIDTH;
import static io.excaliburfrc.robot.Constants.DriveConstants.WHEEL_RADIUS;
import static io.excaliburfrc.robot.Constants.DriveConstants.kA_ang;
import static io.excaliburfrc.robot.Constants.DriveConstants.kA_lin;
import static io.excaliburfrc.robot.Constants.DriveConstants.kV_ang;
import static io.excaliburfrc.robot.Constants.DriveConstants.kV_lin;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.lib.sim.AutoSimDrivetrain;

public class Drivetrain extends SubsystemBase {
  @AutoSimDrivetrain.LeftMotor private final MotorControllerGroup leftMotors;
  @AutoSimDrivetrain.RightMotor private final MotorControllerGroup rightMotors;
  @AutoSimDrivetrain.LeftEncoder private final Encoder leftEncoder;
  @AutoSimDrivetrain.RightEncoder private final Encoder rightEncoder;
  @AutoSimDrivetrain.Gyro private final AHRS gyro;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  private final Field2d field = new Field2d();

  private final AutoSimDrivetrain sim;

  public Drivetrain() {
    leftMotors = new MotorControllerGroup(new PWMSparkMax(1), new PWMSparkMax(2));
    rightMotors = new MotorControllerGroup(new PWMSparkMax(3), new PWMSparkMax(4));
    leftEncoder = new Encoder(1, 2);
    rightEncoder = new Encoder(3, 4);
    gyro = new AHRS(SPI.Port.kMXP);

    drive = new DifferentialDrive(leftMotors, rightMotors);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    sim =
        AutoSimDrivetrain.Initialize(
            this,
            new DifferentialDrivetrainSim(
                LinearSystemId.identifyDrivetrainSystem(kV_lin, kA_lin, kV_ang, kA_ang),
                DCMotor.getNEO(2),
                GEARING,
                TRACK_WIDTH,
                WHEEL_RADIUS,
                null));
  }

  public void arcadeDrive(double xSpeed, double yRotation) {
    drive.arcadeDrive(xSpeed, yRotation);
  }

  @Override
  public void simulationPeriodic() {
    sim.simulationPeriodic();
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("field", field);
  }
}
