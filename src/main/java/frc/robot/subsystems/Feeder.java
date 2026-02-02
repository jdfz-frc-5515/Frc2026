package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    public enum Speed {
        FEED(120);

        private final double rpm;
        private final double mech2sensorRatio = 27.0;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity feedAngularVelocity() {
            return RPM.of(rpm);
        }
        public AngularVelocity motorAngularVelocity() {
            return RPM.of(rpm*mech2sensorRatio);
        }
    }
    private final int feedMotorCANID = 0; // default values for CAN ID and Bus
    private final CANBus feedMotorCANBus= new CANBus("canbus");
    private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Feeder() {
        motor = new TalonFX(feedMotorCANID, feedMotorCANBus);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(50))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.01)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0.04)
                    .withKV(0.115)
                    .withKA(0.0)
            );
        
        motor.getConfigurator().apply(config);
    }

    public void set(Speed speed) {
        motor.setControl(
            velocityRequest
                .withVelocity(speed.motorAngularVelocity())
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public Command feedCommand() {
        return startEnd(() -> set(Speed.FEED), () -> setPercentOutput(0));
    }
}