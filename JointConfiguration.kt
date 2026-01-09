package net.tecdroid.subsystems.joint

import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import net.tecdroid.mechanical.Reduction
import net.tecdroid.util.ControlGains
import net.tecdroid.util.RotationalDirection
//This are all the library

data class JointConfig(
    val motorControllerId: Int,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,
    val neutralMode: SparkBaseConfig.IdleMode,
    val gearRatio: Reduction,
    val controlGains: ControlGains,
)

val jointConfig : JointConfig = JointConfig(
    motorControllerId = 2,
    motorDirection = RotationalDirection.Counterclockwise,
    motorCurrentLimit = Units.Amps.of(40.0),
    neutralMode = SparkBaseConfig.IdleMode.kBrake,
    Reduction(3.1),
    ControlGains(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),

)