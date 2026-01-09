package net.tecdroid.subsystems.joint

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkBase
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import net.tecdroid.util.RotationalDirection
import net.tecdroid.util.volts
import edu.wpi.first.math.MathUtil


// Neos
class Joint(val config: JointConfig) {
    val jointMotor = SparkMax(config.motorControllerId, SparkLowLevel.MotorType.kBrushless)
    //val positionJoint = RelativeEncoder(jointMotor) ,

    fun setPosition(angle: Angle) {

       val clampedPosition = MathUtil.clamp(angle.`in`(Units.Degrees), 0.0, 90.0)
        jointMotor.closedLoopController.setReference(clampedPosition.`in`(Units.Rotations), SparkBase.ControlType.kPosition)
    }

    init {
        val sparkMaxConfig = SparkMaxConfig()


        sparkMaxConfig.smartCurrentLimit(config.motorCurrentLimit.`in`(Units.Amps).toInt())
        sparkMaxConfig.inverted(config.motorDirection != RotationalDirection.Counterclockwise)
        sparkMaxConfig.idleMode(config.neutralMode)
        sparkMaxConfig.closedLoop.p(config.controlGains.p)
        sparkMaxConfig.closedLoop.i(config.controlGains.i)
        sparkMaxConfig.closedLoop.d(config.controlGains.d)

        jointMotor.clearFaults()

        jointMotor.configure(sparkMaxConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters
        )
    }
}

/*
class Joint(val config : JointConfig) {
    val talon = TalonFX(0)
    init {
        val talonFXConfig = TalonFXConfiguration()

        fun setPosition(angle: Angle) {
            talon.setControl(VoltageOut(voltage))
        }

        fun stop() {
            setVoltage(0.0.volts)
        }



with(talonFXConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.motorDirection.toInvertedValue())

            CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorCurrentLimit)

            Slot0.withKP(config.controlGains.p)
                .withKI(config.controlGains.i)
                .withKD(config.controlGains.d)
            }
        talon.configurator.apply(talonFXConfig)
        }
    }
}
 */