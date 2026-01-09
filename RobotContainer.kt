package net.tecdroid.core

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import net.tecdroid.subsystems.Intake.Intake
import net.tecdroid.subsystems.Intake.intakeConfig
import net.tecdroid.subsystems.joint.Joint
import net.tecdroid.subsystems.joint.jointConfig


class RobotContainer {
    val control = CommandXboxController(0)

    val intake: Intake = Intake(intakeConfig)
    val jointMotor: Joint = Joint(jointConfig)


    init {
            control.a()
                .onTrue(InstantCommand({ intake.setVoltage(Units.Volts.of(12.0)) }))
                .onFalse(InstantCommand({ intake.stop() }))
            control.y()
                .onTrue(InstantCommand({jointMotor.se)
    };

    fun teleopInit() { }

    fun robotPeriodic() {}



}
