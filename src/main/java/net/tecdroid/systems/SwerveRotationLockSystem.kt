package net.tecdroid.systems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.drivetrain.Drive
import net.tecdroid.constants.Constants
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.util.ControlGains
import net.tecdroid.util.degrees
import net.tecdroid.util.meters
import kotlin.math.max
import kotlin.math.min

enum class LockPositions {
    CoralStation,
    Proccesor
}

class SwerveRotationLockSystem (private val swerve: Drive, private val controller: CompliantXboxController){
    private val thetaGains = ControlGains(0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    private val thetaPIDController = PIDController(thetaGains.p, thetaGains.i, thetaGains.d)

    init {
        val tab = Shuffleboard.getTab("angleTab")
        tab.addDouble("angle", { getLimitedYaw() })

        thetaPIDController.enableContinuousInput(0.0, 360.0)
    }

    private fun getLimitedYaw(): Double {
        var limitedYaw: Double = swerve.rotation.degrees % 360
        if (limitedYaw < 0) {
            limitedYaw += 360.0
        }
        return limitedYaw
    }

    private fun clamp(max: Double, min: Double, v: Double): Double {
        return max(min, min(max, v))
    }

    // Change the target angle depending on the side on where the robot is
    fun getCoralStationAngleAccordingToRobotPosition() : Angle {
        val FIELD_WIDTH = 8.21.meters
        val y = swerve.poseEstimator.estimatedPosition.y

        // If is greater, the robot is on the right side
        if (y > FIELD_WIDTH.baseUnitMagnitude() / 2.0) {
            return 215.0.degrees
        } else {
            return 135.0.degrees
        }
    }


    fun lockRotationCMD(position: LockPositions) : Command = Commands.run({
        // Controller velocities
        val vx = MathUtil.applyDeadband(controller.leftY, 0.05) * 0.85
        val vy = MathUtil.applyDeadband(controller.leftX, 0.05) * 0.85

        val targetXVelocity = MetersPerSecond.of(swerve.maxLinearSpeedMetersPerSec) * vx
        val targetYVelocity = MetersPerSecond.of(swerve.maxLinearSpeedMetersPerSec) * vy

        // PID theta velocity

        // Get the target angle according to the target position
        val targetAngle = when (position) {
            LockPositions.CoralStation -> getCoralStationAngleAccordingToRobotPosition()
            LockPositions.Proccesor -> 45.0.degrees
        }
        val wFactor = clamp(1.0, -1.0, thetaPIDController.calculate(getLimitedYaw(), targetAngle.`in`(Units.Degrees)))

        val targetWVelocity = Units.DegreesPerSecond.of(Math.toDegrees(swerve.maxAngularSpeedRadPerSec.times(0.75)) * wFactor)

        swerve.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            targetXVelocity, targetYVelocity, targetWVelocity,
            if (Constants.isFlipped.invoke()) swerve.rotation.plus(Rotation2d(Math.PI)) else swerve.rotation))
    }, swerve)
}