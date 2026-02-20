package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {
    
    public enum Position {
        OUT(OUTPOS),
        IN(INPOS);
        private final double position;
        private final double mech2sensorRatio = GearRatio;

        private Position(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
        public double motorPosition() {
            return position * mech2sensorRatio;
        }
    }
    public enum ExtenderSpeed {
        SLOWOUT(slowOut),
        SLOWIN(-slowOut);
        private final double rpm;
        private final double mech2sensorRatio = GearRatio;

        private ExtenderSpeed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity feedAngularVelocity() {
            return RPM.of(rpm);
        }
        public AngularVelocity motorAngularVelocity() {
            return RPM.of(rpm*mech2sensorRatio);
        }
    }
    private final int extenderMotorCANID = frc.robot.Constants.ExtenderConstants.EXTENDER_CAN_ID; // default values for CAN ID and Bus
    private final CANBus extenderMotorCANBus= new CANBus("rio");
    private final TalonFX motor;
    private static double GearRatio = frc.robot.Constants.ExtenderConstants.GEAR_RATIO;
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private static double slowOut = frc.robot.Constants.ExtenderConstants.SLOW_OUT_RPM;
    // PID S V A constants for extender slot0 are defined in Constants.ExtenderConstants
    private static final double EXT_KP = frc.robot.Constants.ExtenderConstants.KP;
    private static final double EXT_KI = frc.robot.Constants.ExtenderConstants.KI;
    private static final double EXT_KD = frc.robot.Constants.ExtenderConstants.KD;
    private static final double EXT_KS = frc.robot.Constants.ExtenderConstants.KS;
    private static final double EXT_KV = frc.robot.Constants.ExtenderConstants.KV;
    private static final double EXT_KA = frc.robot.Constants.ExtenderConstants.KA;
    private static double INPOS = frc.robot.Constants.ExtenderConstants.IN_POS;
    private static double OUTPOS = frc.robot.Constants.ExtenderConstants.OUT_POS;
    public Extender() {
        motor = new TalonFX(extenderMotorCANID, extenderMotorCANBus);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
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
                    .withKP(EXT_KP)
                    .withKI(EXT_KI)
                    .withKD(EXT_KD)
                    .withKS(EXT_KS)
                    .withKV(EXT_KV)
                    .withKA(EXT_KA)
            );
        
        motor.getConfigurator().apply(config);
    }

    public void setPosition(Position position) {
        motor.setControl(
            positionRequest.withPosition(position.motorPosition())
        );
    }

    public void setVelocity(ExtenderSpeed speed) {
        motor.setControl(
            velocityRequest.withVelocity(speed.motorAngularVelocity())
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public double getCurrentMotorPosition(){
        return motor.getPosition().getValueAsDouble();
    }
}
