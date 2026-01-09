package net.tecdroid.subsystems.Intake

import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.util.ControlGains
import net.tecdroid.util.RotationalDirection

data class IntakeConfig(
    val motorControllerId: Int,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,
    val neutralMode: SparkBaseConfig.IdleMode,
    val gearRatio: Reduction,
    val controlGains: ControlGains
)

val intakeConfig: IntakeConfig = IntakeConfig(
    motorControllerId = 1,
    motorDirection = RotationalDirection.Clockwise,
    motorCurrentLimit = Units.Amps.of(40.0),
    neutralMode = SparkBaseConfig.IdleMode.kBrake,
    Reduction(9.1),
    controlGains = ControlGains(0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0)
)