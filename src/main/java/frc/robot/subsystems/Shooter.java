package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Simple two-motor Flywheel/Shooter subsystem.
 *
 * - Primary motor is controlled directly.
 * - Secondary motor follows the primary and can be inverted.
 * - Provides velocity (RPM) control and percent output control, plus getters.
 *
 * Usage:
 *   var shooter = new Shooter(primaryId, followerId, "canbus", true);
 *   shooter.setVelocityRpm(5000);
 */
public class Shooter extends SubsystemBase {
    private final TalonFX primary;
    private final TalonFX follower;
    private final int PrimaryCanID = ShooterConstants.PRIMARY_CAN_ID;
    private final int FollowerCanID = ShooterConstants.FOLLOWER_CAN_ID;
    private final CANBus shooterCanBus = new CANBus("rio");
    // PID/SVA constants for shooter slot0
    private static final double SHOOT_KP = ShooterConstants.KP;
    private static final double SHOOT_KI = ShooterConstants.KI;
    private static final double SHOOT_KD = ShooterConstants.KD;
    private static final double SHOOT_KS = ShooterConstants.KS;
    private static final double SHOOT_KV = ShooterConstants.KV;
    private static final double SHOOT_KA = ShooterConstants.KA;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    /**
     * Construct a Shooter with explicit IDs and bus name.
     * followerInverted: true means follower will be inverted relative to primary.
     */
    public Shooter() {
    // IDs are used to construct TalonFX instances below; no need to store them separately
        this.primary = new TalonFX(PrimaryCanID, shooterCanBus);
        this.follower = new TalonFX(FollowerCanID, shooterCanBus);

        // Apply configuration — primary and follower can have different motor inversion
        primary.getConfigurator().apply(getConfiguration());
        follower.getConfigurator().apply(getConfiguration());
        follower.setControl(new Follower(primary.getDeviceID(), MotorAlignmentValue.Opposed));
    }
    /**
     * Set flywheel speed in RPM (motor shaft RPM). This issues a VelocityVoltage control.
     */
    public void setVelocityRpm(double rpm) {
        AngularVelocity ang = RPM.of(rpm);
        primary.setControl(velocityRequest.withVelocity(ang));
    }

    /**
     * Set open-loop percent output (-1..1).
     */
    public void setPercentOutput(double percent) {
        primary.setControl(voltageRequest.withOutput(Volts.of(-percent * 12.0)));
    }

    /**
     * Build a TalonFXConfiguration with the requested motor inversion.
     * This centralizes tuning parameters and keeps primary/follower configs consistent.
     */
    private TalonFXConfiguration getConfiguration() {
        return new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(SHOOT_KP)
                    .withKI(SHOOT_KI)
                    .withKD(SHOOT_KD)
                    .withKS(SHOOT_KS)
                    .withKV(SHOOT_KV)
                    .withKA(SHOOT_KA)
            );
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Primary Velocity", getPrimaryVelocity());
        SmartDashboard.putNumber("Follower Velocity", getFollowerVelocity());
        SmartDashboard.putNumber("Average Velocity", getAverageVelocity());
    }

    /**
     * Stop the flywheel (set 0 output).
     */
    public void stop() {
        setPercentOutput(0.0);
    }

    /**
     * Return primary motor's rotor velocity (rotations per second or library-specific unit).
     * Caller may need to convert units to RPM: multiply by 60 if value is rotations/sec.
     */
    /** Return primary motor velocity (rotations per second, vendor-specific unit). */
    public double getPrimaryVelocity() {
        return primary.getVelocity().getValueAsDouble();
    }

    /** Return follower motor velocity (rotations per second, vendor-specific unit). */
    public double getFollowerVelocity() {
        return follower.getVelocity().getValueAsDouble();
    }

    /** Average velocity of primary and follower (rotations per second). */
    public double getAverageVelocity() {
        return (getPrimaryVelocity() + getFollowerVelocity()) / 2.0;
    }
}
