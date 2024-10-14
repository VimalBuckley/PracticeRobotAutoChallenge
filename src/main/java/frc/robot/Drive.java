package frc.robot;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private TalonFX rightMotor;
    private TalonFX leftMotor;
    private NavX gyro;
    private VelocityVoltage leftVel;
    private VelocityVoltage rightVel;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;
    
    public Drive() {
        rightMotor = new TalonFX(1);
        leftMotor = new TalonFX(2);
        gyro = new NavX(Port.kMXP);
        leftVel = new VelocityVoltage(0);
        rightVel = new VelocityVoltage(0);
        kinematics = new DifferentialDriveKinematics(0.6);
        odometry = new DifferentialDriveOdometry(
            gyro.getAngle(), 
            getDistance().leftMeters,
            getDistance().rightMeters
        );
        AutoBuilder.configureLTV(
            this::getPose, 
            this::resetPose, 
            this::getSpeeds, 
            this::drive, 
            0.02, 
            new ReplanningConfig(), 
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
            this
        );
    }

    private Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    private void resetPose(Pose2d newPose) {
        odometry.resetPosition(gyro.getAngle(), getDistance(), newPose);
    }

    private ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getVelocity());
    }

    private void drive(ChassisSpeeds speeds) {
        setVelocity(kinematics.toWheelSpeeds(speeds));
    }

    private DifferentialDriveWheelPositions getDistance() {
        return new DifferentialDriveWheelPositions(
            leftMotor.getPosition().getValueAsDouble(),
            rightMotor.getPosition().getValueAsDouble()
        );
    }

    private DifferentialDriveWheelSpeeds getVelocity() {
        return new DifferentialDriveWheelSpeeds(
            leftMotor.getVelocity().getValueAsDouble(), 
            rightMotor.getVelocity().getValueAsDouble()
        );
    }

    private void setVelocity(DifferentialDriveWheelSpeeds velocity) {
        leftMotor.setControl(leftVel.withVelocity(velocity.leftMetersPerSecond));
        rightMotor.setControl(rightVel.withVelocity(velocity.rightMetersPerSecond));
    }
}
