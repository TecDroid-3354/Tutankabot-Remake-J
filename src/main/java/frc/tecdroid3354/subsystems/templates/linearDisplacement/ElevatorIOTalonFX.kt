package frc.tecdroid3354.subsystems.templates.linearDisplacement

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MutDistance
import frc.tecdroid3354.constants.SubsystemsControlGains
import frc.tecdroid3354.constants.SubsystemsControlRequests
import frc.tecdroid3354.constants.SubsystemsMotionTargets
import frc.tecdroid3354.constants.SubsystemsMovementLimits
import frc.tecdroid3354.utils.devices.OpTalonFX
import frc.tecdroid3354.subsystems.templates.linearDisplacement.ElevatorConstants.Identification
import frc.tecdroid3354.subsystems.templates.linearDisplacement.ElevatorConstants.Mechanical
import frc.tecdroid3354.utils.interfaces.MotorIO
import java.util.Optional

/**
 * Hardware layer for TalonFX motor controllers. Only file where [com.ctre.phoenix6.hardware.TalonFX]
 * motors are instantiated for this subsystem.
 *
 * **NOTE:** All methods implemented from the interface will inherit their comments. It is not necessary to repeat
 * those comments here.
 */
class ElevatorIOTalonFX : ElevatorIO {
    // Make sure to configure it.
    private val leadMotorController : OpTalonFX = OpTalonFX(Identification.LEAD_MOTOR_ID,
                                                            Identification.ELEVATOR_CANBUS_NAME,
                                                            ElevatorConstants.PhoenixMotorConfiguration.ENABLE_HIGHER_FREQUENCY)
    // Make sure to configure it and set it as follower.
    private val followerMotorController : OpTalonFX = OpTalonFX(Identification.FOLLOWER_MOTOR_ID,
                                                            Identification.ELEVATOR_CANBUS_NAME,
                                                            ElevatorConstants.PhoenixMotorConfiguration.ENABLE_HIGHER_FREQUENCY)
    private val elevatorTargetDisplacement : MutDistance = Meters.mutable(0.0)
    private val elevatorManualTargetDisplacement: MutDistance = Meters.mutable(0.0)

    override fun updateElevatorInputs(inputs: ElevatorIO.ElevatorIOInputs,
                                      leadMotorInputs: MotorIO.MotorIOInputs, followerMotorInputs: MotorIO.MotorIOInputs) {
        inputs.elevatorDisplacement.mut_replace(leadMotorController.getMotorToLinearSubsystemDisplacement(
            Mechanical.REDUCTION, Mechanical.SPROCKET
        ))
        inputs.elevatorTargetDisplacement.mut_replace(elevatorTargetDisplacement)
        inputs.elevatorManualTargetDisplacement.mut_replace(elevatorManualTargetDisplacement)

        leadMotorController.updateInputs(leadMotorInputs)
        followerMotorController.updateInputs(followerMotorInputs)
    }

    override fun updateElevatorManualDisplacement(newElevatorManualDisplacement: Distance) {
        this.elevatorManualTargetDisplacement.mut_replace(newElevatorManualDisplacement)
    }

    override fun updateElevatorMotorsControlGains(slot: Int) {
        // Make sure the selected slot is either 0, 1, or 2
        val validatedSlot = MathUtil.clamp(slot, 0, 2)
        // Clone the initial config
        val newMotorsConfig = ElevatorConstants.PhoenixMotorConfiguration.initialMotorsConfiguration.clone()

        when (validatedSlot) { // Update the corresponding Slot Configs
            0 -> { // Note that only primary gains are declared, hence is the only one in use for all slots
                newMotorsConfig.Slot0 = SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.updatePhoenixSlot0Configs()
            }
            1 -> {
                newMotorsConfig.Slot1 = SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.updatePhoenixSlot1Configs()
            }
            else -> { // Can assume else {} branch to be 2, since we're using the validatedSlot
                newMotorsConfig.Slot2 = SubsystemsControlGains.ELEVATOR_MOTOR_PRIMARY_GAINS.updatePhoenixSlot2Configs()
            }
        }

        leadMotorController.applySlotConfigs(newMotorsConfig.Slot0, newMotorsConfig.Slot1, newMotorsConfig.Slot2)
    }

    override fun setElevatorManualTargetDisplacement(): Runnable {
        return {
            this.elevatorTargetDisplacement.mut_replace(elevatorManualTargetDisplacement)

            leadMotorController.linearSubsystemPositionDynamicRequest(
                controlType = SubsystemsControlRequests.ELEVATOR_CONTROL_TYPE,
                subsystemDisplacement = elevatorManualTargetDisplacement,
                limits = SubsystemsMovementLimits.ELEVATOR_DISPLACEMENT_LIMITS,
                sprocket = Mechanical.SPROCKET,
                reduction = Mechanical.REDUCTION,
                // Note how the SECONDARY motion targets are used when using manual target displacement
                newSubsystemMotionTargets = SubsystemsMotionTargets.ELEVATOR_SECONDARY_MOTION_TARGETS
            )
        }
    }

    override fun setElevatorTargetDisplacement(elevatorTargetDisplacement: Distance): Runnable {
        return {
            this.elevatorTargetDisplacement.mut_replace(elevatorTargetDisplacement)

            leadMotorController.linearSubsystemPositionDynamicRequest(
                controlType = SubsystemsControlRequests.ELEVATOR_CONTROL_TYPE,
                subsystemDisplacement = elevatorTargetDisplacement,
                limits = SubsystemsMovementLimits.ELEVATOR_DISPLACEMENT_LIMITS,
                sprocket = Mechanical.SPROCKET,
                reduction = Mechanical.REDUCTION,
                newSubsystemMotionTargets = SubsystemsMotionTargets.ELEVATOR_PRIMARY_MOTION_TARGETS
            )
        }
    }

    override fun stopElevator(): Runnable {
        return { leadMotorController.stopMotor() }
    }

    override fun coastElevatorMotors(): Runnable {
        return {
            leadMotorController.coast()
            followerMotorController.coast()
        }
    }

    override fun brakeElevatorMotors(): Runnable {
       return {
            leadMotorController.brake()
            followerMotorController.brake()
       }
    }

    override fun initialMotorConfiguration() {
        leadMotorController.applyFullConfigAndClearFaults(ElevatorConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)
        followerMotorController.applyFullConfigAndClearFaults(ElevatorConstants.PhoenixMotorConfiguration.initialMotorsConfiguration)

        followerMotorController.follow(
            leadMotorController.getMotorInstance(),
            ElevatorConstants.PhoenixMotorConfiguration.followerMotorAlignment)
    }
}