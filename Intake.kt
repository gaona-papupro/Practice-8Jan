package net.tecdroid.subsystems.Intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import net.tecdroid.util.RotationalDirection
import net.tecdroid.util.volts

/*
NEO motor
class Intake(val config: IntakeConfig) {
    val spark = SparkMax(config.motorControllerId, MotorType.kBrushless)

    fun setVoltage(voltage: Voltage) {
        spark.setVoltage(voltage)
    }

    fun stop() {
        setVoltage(Units.Volts.of(0.0))
    }

    init {
        val sparkMaxConfig = SparkMaxConfig()

        sparkMaxConfig.smartCurrentLimit(config.motorCurrentLimit.`in`(Units.Amps).toInt())
        sparkMaxConfig.inverted(config.motorDirection != RotationalDirection.Counterclockwise)
        sparkMaxConfig.idleMode(config.neutralMode)

        spark.clearFaults()

        spark.configure(
            sparkMaxConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters
        )
    }
} */
//Talon
class Intake(val config : IntakeConfig) {
    val talon = TalonFX(0  )

    fun setVoltage(voltage: Voltage) {
        talon.setControl(VoltageOut(voltage))
    }

    fun stop() {
        setVoltage(0.0.volts)
    }
    init {
        val talonFXConfig = TalonFXConfiguration()

        with(talonFXConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.motorDirection.toInvertedValue())

            CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorCurrentLimit)

        }

        talon.configurator.apply(talonFXConfig)
    }
}


