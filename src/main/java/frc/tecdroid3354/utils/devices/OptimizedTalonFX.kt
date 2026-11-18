package frc.tecdroid3354.utils.devices

import frc.tecdroid3354.utils.PhoenixUtil.*

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.Slot2Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoTorqueCurrentFOC
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.AngularVelocityUnit
import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.LinearVelocityUnit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage
import frc.tecdroid3354.constants.RobotConstants
import frc.tecdroid3354.constants.RobotMode
import frc.tecdroid3354.utils.Sprocket
import frc.tecdroid3354.utils.controlProfiles.AngularMotionTargets
import frc.tecdroid3354.utils.controlProfiles.LinearMotionTargets
import frc.tecdroid3354.utils.interfaces.MotorIO
import frc.tecdroid3354.utils.hertz
import frc.tecdroid3354.utils.mechanical.Reduction
import frc.tecdroid3354.utils.rotations
import frc.tecdroid3354.utils.rotationsPerSecond
import frc.tecdroid3354.utils.safety.MeasureLimits
import frc.tecdroid3354.utils.seconds

/**
 * Contains all MotionMagic Request Types used in the team for position control.
 */
enum class OpPositionControlRequests {
    POSITION_VOLTAGE,               // Unlicensed
    POSITION_TORQUE,                // Licensed
    POSITION_EXPO_TORQUE,           // Licensed
    POSITION_DYNAMIC_TORQUE,        // Licensed
    POSITION_DYNAMIC_EXPO_TORQUE,   // Licensed
}

/**
 * Contains all the MotionMagic Request Types used in the team for velocity control.
 */
enum class OpVelocityControlRequests {
    VELOCITY_VOLTAGE,   // Unlicensed
    VELOCITY_TORQUE,    // Licensed
}

//
// DECLARES REFRESH RATE OF PRIMARY AND SECONDARY SIGNALS
//
private val DEFAULT_SIGNAL_FREQUENCY    : Frequency = 100.0.hertz
private val HIGHER_SIGNAL_FREQUENCY     : Frequency = 250.0.hertz
private val LOWER_SIGNAL_FREQUENCY      : Frequency = 50.0.hertz

//
// TIMEOUT FOR APPLYING CONFIGS
//
private val FULL_CONFIG_APPLYING_TIMEOUT    : Time = 0.25.seconds
private val SINGLE_CONFIG_APPLYING_TIMEOUT  : Time = 0.02.seconds

private const val MAX_APPLY_ATTEMPTS_FULL_CONFIG    : Int = 5
private const val MAX_APPLY_ATTEMPTS_SINGLE_CONFIG  : Int = 2


/**
 * A Phoenix6 TalonFX utility for easier control across subsystems.
 *
 * The intention is to pass this wrapper all your **SUBSYSTEM** targets - as marked in the names of the methods -, and
 * forget about all unit conversion and safety, which is done here.
 *
 * Auxiliary signals and unit conversion methods are included for telemetry.
 *
 * This class also optimizes CANBus utilization by only keeping relevant signals active. Motor configuration
 * and follower requests methods are also included.
 * @param useHigherFrequency Sets position and velocity signals to update at 250Hz instead of the usual 100Hz
 */
class OpTalonFX(private val id: Int, private val canBusName: String = "rio", private val useHigherFrequency: Boolean): MotorIO {
    private val motor = KrakenMotors.createDefaultTalon(id ,canBusName)
    private var isFollower: Boolean = false

    private val voltageOutRequest = VoltageOut(0.0) // For SysId Only. Do NOT use with Torque requests.

    //
    // POSITION BASED REQUESTS
    //
    private val motionMagicPositionVoltageRequest = MotionMagicVoltage(0.0) // For Unlicensed use only

    private val motionMagicPositionTorqueRequest = MotionMagicTorqueCurrentFOC(0.0) // Requires license
    private val motionMagicPositionExpoTorqueRequest = MotionMagicExpoTorqueCurrentFOC(0.0) // Requires License
    private val dynamicMotionMagicPositionTorqueRequest = DynamicMotionMagicTorqueCurrentFOC(0.0, 0.0, 0.0)
    private val dynamicMotionMagicPositionExpoTorqueRequest = DynamicMotionMagicExpoTorqueCurrentFOC(0.0, 0.0, 0.0)

    //
    // VELOCITY BASED REQUESTS
    //
    private val motionMagicVelocityVoltageRequest = MotionMagicVelocityVoltage(0.0) // For Unlicensed use only
    private val motionMagicVelocityTorqueRequest = MotionMagicVelocityTorqueCurrentFOC(0.0) // Requires License

    // Class-level exception to include the ID of the motor.
    private fun commandToFollowerException() = IllegalCallerException("Tried to command a follower TalonFX [$id]. Use the lead TalonFX.")
    private fun dynamicToNonDynamicMethod() = IllegalCallerException("Tried a dynamic request in a non-dynamic method. Called for TalonFX [$id].")
    private fun nonDynamicToDynamicMethod() = IllegalCallerException("Tried a non-dynamic request in a dynamic method. Called for TalonFX [$id].")
    private fun missingParametersForControlType() = IllegalCallerException("Missing parameter for control type. Called for TalonFX [$id].")

    init {
        optimizeMotorCan() // This method should not be called anywhere else.
    }

    override fun updateInputs(inputs: MotorIO.MotorIOInputs) {
        inputs.isConnected = getIsConnected()

        inputs.closedLoopReference.mut_replace(getClosedLoopTargetPosition())
        inputs.closedLoopReferenceSlope.mut_replace(getClosedLoopTargetVelocity())

        inputs.currentPosition.mut_replace(getPosition())
        inputs.currentVelocity.mut_replace(getVelocity())
        inputs.currentAcceleration.mut_replace(getAcceleration())

        inputs.currentTemperature.mut_replace(getTemperature())
        inputs.outputVoltage.mut_replace(getOutputVoltage())
        inputs.supplyCurrent.mut_replace(getSupplyCurrent())
        inputs.torqueCurrent.mut_replace(getTorqueCurrent())
    }

    //
    // Getters for relevant signals
    //
    fun getPosition(): Angle { return motor.position.value }
    fun getVelocity(): AngularVelocity { return motor.velocity.value }
    fun getAcceleration(): AngularAcceleration { return motor.acceleration.value }
    fun getTemperature(): Temperature { return motor.deviceTemp.value }
    fun getOutputVoltage(): Voltage { return motor.motorVoltage.value }
    fun getSupplyCurrent(): Current { return motor.supplyCurrent.value }
    fun getTorqueCurrent(): Current { return motor.torqueCurrent.value }
    fun getPower(): Double { return motor.get() }
    fun getIsConnected(): Boolean { return if(RobotConstants.ROBOT_MODE == RobotMode.SIM) true else motor.isConnected }

    fun getMotorId(): Int { return motor.deviceID }
    fun getMotorInstance(): TalonFX { return motor }

    //
    // Closed Loop References (motor internally-calculated target position and velocity)
    //
    fun getClosedLoopTargetPosition(): Angle { return motor.closedLoopReference.value.rotations }
    fun getClosedLoopTargetVelocity(): AngularVelocity { return motor.closedLoopReferenceSlope.value.rotationsPerSecond }

    // --------------- ------- ------- --------------- //
    // --------------- REQUEST METHODS --------------- //
    // --------------- ------- ------- --------------- //

    /**
     * Non-MotionMagic request. Uses [VoltageOut]; intended only for Sys-Id usage.
     * Warning -> SysId will not give appropriate gains for torque-based subsystems.
     */
    fun sysIdVoltageRequest(voltage: Voltage) {
        if (isFollower) { throw commandToFollowerException() }

        motor.setControl(voltageOutRequest
            .withOutput(voltage)
        )
    }

    /** Will throw an [Exception] if commanded to a follower motor. Uses [MotionMagicVoltage] */
    private fun positionVoltageRequest(position: Angle, slot: Int) {
        if (isFollower) { throw commandToFollowerException() }

        motor.setControl(motionMagicPositionVoltageRequest
            .withPosition(position)
            .withSlot(slot)
        )
    }

    /** Will throw an [Exception] if commanded to a follower motor. Uses [MotionMagicTorqueCurrentFOC]. Requires Phoenix Pro */
    private fun positionTorqueRequest(position: Angle, slot: Int) {
        if (isFollower) { throw commandToFollowerException() }

        motor.setControl(motionMagicPositionTorqueRequest
            .withPosition(position)
            .withSlot(slot)
        )
    }

    /** Will throw an [Exception] if commanded to a follower motor. Uses [MotionMagicExpoTorqueCurrentFOC]. Requires Phoenix Pro */
    private fun positionExpoTorqueRequest(position: Angle, slot: Int) {
        if (isFollower) { throw commandToFollowerException() }

        motor.setControl(motionMagicPositionExpoTorqueRequest
            .withPosition(position)
            .withSlot(slot)
        )
    }

    /** Will throw an [Exception] if commanded to a follower motor. Uses [DynamicMotionMagicTorqueCurrentFOC]. Requires Phoenix Pro */
    private fun positionDynamicTorque(position: Angle, velocity: AngularVelocity, acceleration: AngularAcceleration,
                                      jerk: Velocity<AngularAccelerationUnit>,  slot: Int) {
        if (isFollower) { throw commandToFollowerException() }

        motor.setControl(dynamicMotionMagicPositionTorqueRequest
            .withPosition(position)
            .withVelocity(velocity)
            .withAcceleration(acceleration)
            .withJerk(jerk)
            .withSlot(slot)
        )
    }

    /** Will throw an [Exception] if commanded to a follower motor. Uses [DynamicMotionMagicExpoTorqueCurrentFOC]. Requires Phoenix Pro */
    private fun positionDynamicExpoTorque(position: Angle, velocity: AngularVelocity, expoKV: Double,
                                          expoKA: Double, slot: Int) {
        if (isFollower) { throw commandToFollowerException() }

        motor.setControl(dynamicMotionMagicPositionExpoTorqueRequest
            .withPosition(position)
            .withVelocity(velocity)
            .withKV(expoKV)
            .withKA(expoKA)
            .withSlot(slot)
        )
    }

    /** Will throw an [Exception] if commanded to a follower motor. Uses [MotionMagicVelocityVoltage]. */
    private fun velocityVoltageRequest(velocity: AngularVelocity, slot: Int) {
        if (isFollower) { throw commandToFollowerException() }

        motor.setControl(motionMagicVelocityVoltageRequest
            .withVelocity(velocity)
            .withSlot(slot))
    }

    /** Will throw an [Exception] if commanded to a follower motor. Uses [MotionMagicVelocityTorqueCurrentFOC]. Requires Phoenix Pro */
    private fun velocityTorqueRequest(velocity: AngularVelocity, slot: Int) {
        if (isFollower) { throw commandToFollowerException() }

        motor.setControl(motionMagicVelocityTorqueRequest
            .withVelocity(velocity)
            .withSlot(slot)
        )
    }

    // --------------- ------- ------- ------- --------------- //
    // --------------- REQUEST MAPPING METHODS --------------- //
    // --------------- ------- ------- ------- --------------- //

    /**
     * Handles all [OpPositionControlRequests] by calling the corresponding method.
     *
     * **USE NAMED ARGUMENTS WHEN CALLING THIS METHOD TO AVOID CONFUSION**
     *
     * If you don't give a value to a parameter and the selected Control Type requires it,
     * an [Exception] will be triggered and the program will stop.
     *
     * This expects to be called from [angularSubsystemPositionRequest], [angularSubsystemPositionDynamicRequest], and
     * [angularSubsystemVelocityRequest] only. See their documentation for instructions on how to use.
     * Linear subsystems convert all linear measures to angular and call the aforementioned methods.
     */
    private fun handlePositionRequest(requestType: OpPositionControlRequests,
                              position: Angle, slot: Int,
                              velocity: AngularVelocity? = null, acceleration: AngularAcceleration? = null,
                              jerk: Velocity<AngularAccelerationUnit>? = null,
                              expoKV: Double? = null, expoKA: Double? = null
                              ) {
        when (requestType) {
            OpPositionControlRequests.POSITION_VOLTAGE              -> { positionVoltageRequest(position, slot) }
            OpPositionControlRequests.POSITION_TORQUE               -> { positionTorqueRequest(position, slot) }
            OpPositionControlRequests.POSITION_EXPO_TORQUE          -> { positionExpoTorqueRequest(position, slot) }
            OpPositionControlRequests.POSITION_DYNAMIC_TORQUE       -> {
                // Throw Exception if required parameters are not given
                require(velocity != null && acceleration != null && jerk != null) { throw missingParametersForControlType() }
                positionDynamicTorque(
                    position, velocity, acceleration, jerk, slot,)
            }
            OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE  -> {
                // Throw Exception if required parameters are not given
                require(velocity != null && expoKV != null && expoKA != null) { throw missingParametersForControlType() }
                positionDynamicExpoTorque(
                    position, velocity, expoKV, expoKA, slot
                )
            }
        }
    }

    /**
     * Calls the corresponding method based on [requestType].
     */
    private fun handleVelocityRequest(requestType: OpVelocityControlRequests, velocity: AngularVelocity, slot: Int) {
        when (requestType) {
            OpVelocityControlRequests.VELOCITY_VOLTAGE  -> { velocityVoltageRequest(velocity, slot) }

            OpVelocityControlRequests.VELOCITY_TORQUE   -> { velocityTorqueRequest(velocity, slot) }
        }
    }

    // --------------- ------- -- --------- ------- --------------- //
    // --------------- REQUEST BY SUBSYSTEM METHODS --------------- //
    // --------------- ------- -- --------- ------- --------------- //

    /**
     * Intended for non-dynamic control types only. Handles limits and subsystem -> motor conversion before calling
     * [handlePositionRequest].
     *
     * The program will throw an [Exception] and stop if you try requesting a dynamic control type through this method,
     * since it does not have the necessary arguments to command them. This is for modularity purposes.
     */
    fun angularSubsystemPositionRequest(controlType: OpPositionControlRequests, subsystemPosition: Angle,
                                        limits: MeasureLimits<AngleUnit>, reduction: Reduction, slot: Int = 0) {
        if (controlType !in listOf( // Make sure user didn't try to use a dynamic request type (not enough parameters)
                OpPositionControlRequests.POSITION_VOLTAGE,
                OpPositionControlRequests.POSITION_TORQUE,
                OpPositionControlRequests.POSITION_EXPO_TORQUE)
            ) { throw dynamicToNonDynamicMethod() }

        val subsystemClampedAngle: Angle = limits.coerceIn(subsystemPosition) as Angle
        val motorTransformedAngle: Angle = reduction.unapply(subsystemClampedAngle)
        handlePositionRequest(
            requestType = controlType,
            position = motorTransformedAngle, slot = slot
        )
    }

    /**
     * Intended for dynamic control types only. Handles limits and subsystem -> motor conversion before calling
     * [handlePositionRequest].
     *
     * **USE NAMED ARGUMENTS WHEN CALLING THIS METHOD TO AVOID CONFUSION**
     *
     * The program will throw an [Exception] and stop if you try requesting a non-dynamic control type through this method,
     * since it might be confusing for new users to try and figure out the extra parameters if not using them.
     * This is for modularity purposes.
     *
     * The program will throw an [Exception] and stop if you do not pass the necessary parameters for your requested
     * control type (i.e., calling [OpPositionControlRequests.POSITION_DYNAMIC_TORQUE] without specifying the subsystem
     * motion targets).
     *
     * @param newSubsystemMotionTargets is Required for [OpPositionControlRequests.POSITION_DYNAMIC_TORQUE] only
     * @param expoVelocity is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     * @param expoKV is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     * @param expoKA is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     */
    fun angularSubsystemPositionDynamicRequest(controlType: OpPositionControlRequests, subsystemPosition: Angle,
                                               limits: MeasureLimits<AngleUnit>, reduction: Reduction,
                                               newSubsystemMotionTargets: AngularMotionTargets? = null,
                                               expoVelocity: AngularVelocity? = null,
                                               expoKV: Double? = null, expoKA: Double? = null, slot: Int = 0) {
        if (controlType !in listOf( // Make sure user didn't try to use a non-dynamic request type
                OpPositionControlRequests.POSITION_DYNAMIC_TORQUE,
                OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE)
        ) { throw nonDynamicToDynamicMethod() }

        val subsystemClampedAngle: Angle = limits.coerceIn(subsystemPosition) as Angle
        val motorTransformedAngle: Angle = reduction.unapply(subsystemClampedAngle)

        if ((newSubsystemMotionTargets != null) && (controlType == OpPositionControlRequests.POSITION_DYNAMIC_TORQUE)) {
            handlePositionRequest(
                requestType = controlType,
                position = motorTransformedAngle, slot = slot,
                velocity = reduction.unapply(newSubsystemMotionTargets.cruiseVelocity),
                acceleration = reduction.unapply(newSubsystemMotionTargets.acceleration),
                jerk = reduction.unapply(newSubsystemMotionTargets.jerk),
            )
        }
        else if ((expoVelocity != null) && (expoKV != null) && (expoKA != null) && (controlType == OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE)) {
            handlePositionRequest(
                requestType = controlType,
                position = motorTransformedAngle, slot = slot,
                velocity = expoVelocity,
                expoKV = expoKV, expoKA = expoKA
            )
        } else { throw missingParametersForControlType() } // Interrupt program to inform devs of parameter emptiness.
    }

    /**
     * This method applies the limits, converts the target from subsystem units to motor units and calls a private
     * method -> [handleVelocityRequest] for the corresponding control request.
     */
    fun angularSubsystemVelocityRequest(controlType: OpVelocityControlRequests, subsystemVelocity: AngularVelocity,
                                        limits: MeasureLimits<AngularVelocityUnit>, reduction: Reduction, slot: Int = 0) {
        val subsystemClampedVelocity: AngularVelocity = limits.coerceIn(subsystemVelocity) as AngularVelocity
        val motorTransformedVelocity: AngularVelocity = reduction.unapply(subsystemClampedVelocity)

        handleVelocityRequest(controlType, motorTransformedVelocity, slot)
    }

    /**
     * Converts all linear measurements to angular and calls [angularSubsystemPositionRequest].
     *
     * Intended for non-dynamic requests only. An [Exception] will be thrown otherwise.
     */
    fun linearSubsystemPositionRequest(controlType: OpPositionControlRequests, subsystemDisplacement: Distance,
                                       limits: MeasureLimits<DistanceUnit>, sprocket: Sprocket, reduction: Reduction,
                                       slot: Int = 0) {
        val linearToAngularDisplacement =  sprocket.linearDisplacementToAngularDisplacement(subsystemDisplacement)
        val linearToAngularLimits = MeasureLimits(
            sprocket.linearDisplacementToAngularDisplacement(limits.minimum as Distance) ..
                    sprocket.linearDisplacementToAngularDisplacement(limits.maximum as Distance))

        angularSubsystemPositionRequest(controlType, linearToAngularDisplacement, linearToAngularLimits, reduction, slot)
    }

    /**
     * Converts all linear measurements to angular and calls [angularSubsystemPositionDynamicRequest].
     *
     * **USE NAMED ARGUMENTS WHEN CALLING THIS METHOD TO AVOID CONFUSION**
     *
     * Intended for dynamic requests only. An [Exception] will be thrown otherwise.
     *
     * @param newSubsystemMotionTargets is Required for [OpPositionControlRequests.POSITION_DYNAMIC_TORQUE] only
     * @param expoVelocity is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     * @param expoKV is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     * @param expoKA is Required for [OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE] only
     */
    fun linearSubsystemPositionDynamicRequest(controlType: OpPositionControlRequests, subsystemDisplacement: Distance,
                                              limits: MeasureLimits<DistanceUnit>, sprocket: Sprocket, reduction: Reduction,
                                              newSubsystemMotionTargets: LinearMotionTargets? = null,
                                              expoVelocity: LinearVelocity? = null,
                                              expoKV: Double? = null, expoKA: Double? = null,
                                              slot: Int = 0) {
        if (controlType !in listOf( // Make sure user didn't try to use a non-dynamic request type
                OpPositionControlRequests.POSITION_DYNAMIC_TORQUE,
                OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE)
        ) { throw nonDynamicToDynamicMethod() }

        val linearToAngularSubsystemDisplacement = sprocket.linearDisplacementToAngularDisplacement(subsystemDisplacement)
        val linearToAngularLimits = MeasureLimits(
            sprocket.linearDisplacementToAngularDisplacement(limits.minimum as Distance) ..
                    sprocket.linearDisplacementToAngularDisplacement(limits.maximum as Distance))

        if ((newSubsystemMotionTargets != null) && (controlType == OpPositionControlRequests.POSITION_DYNAMIC_TORQUE)) {
            // Just makes it angular, but still in mechanism units
            val linearToAngularMotionTargets = newSubsystemMotionTargets.angularMotionTargets(sprocket)

            angularSubsystemPositionDynamicRequest(
                controlType = controlType, subsystemPosition = linearToAngularSubsystemDisplacement,
                limits = linearToAngularLimits, reduction = reduction,
                newSubsystemMotionTargets = linearToAngularMotionTargets,
                slot = slot
            )
        }
        else if ((expoVelocity != null) && (expoKV != null) && (expoKA != null) && (controlType == OpPositionControlRequests.POSITION_DYNAMIC_EXPO_TORQUE)) {
            val angularVelocity = sprocket.linearVelocityToAngularVelocity(expoVelocity)
            angularSubsystemPositionDynamicRequest(
                controlType = controlType, subsystemPosition = linearToAngularSubsystemDisplacement,
                limits = linearToAngularLimits, reduction = reduction,
                expoVelocity = angularVelocity,
                expoKV = expoKV, expoKA = expoKA,
                slot = slot
            )
        } else { throw missingParametersForControlType() } // Interrupt program to inform devs of parameter emptiness.
    }

    /**
     * Converts all linear measurements to angular and calls [angularSubsystemVelocityRequest].
     */
    fun linearSubsystemVelocityRequest(controlType: OpVelocityControlRequests, subsystemVelocity: LinearVelocity,
                                       limits: MeasureLimits<LinearVelocityUnit>, sprocket: Sprocket, reduction: Reduction,
                                       slot: Int = 0) {
        val linearToAngularSubsystemVelocity = sprocket.linearVelocityToAngularVelocity(subsystemVelocity)
        val linearToAngularLimits = MeasureLimits(
            sprocket.linearVelocityToAngularVelocity(limits.minimum as LinearVelocity) ..
                    sprocket.linearVelocityToAngularVelocity(limits.maximum as LinearVelocity))

        angularSubsystemVelocityRequest(controlType, linearToAngularSubsystemVelocity, linearToAngularLimits, reduction, slot)
    }

    // --------------- ---- ---------- ------- --------------- //
    // --------------- UNIT CONVERSION GETTERS --------------- //
    // --------------- ---- ---------- ------- --------------- //

    /**
     * Applies the reduction to get subsystem position
     */
    fun getMotorToAngularSubsystemPosition(reduction: Reduction): Angle {
        return reduction.apply(getPosition())
    }

    /**
     * Applies reduction to get subsystem velocity
     */
    fun getMotorToAngularSubsystemVelocity(reduction: Reduction): AngularVelocity {
        return reduction.apply(getVelocity())
    }

    /**
     * Applies reduction, and then uses the [sprocket] diameter to convert from angular to linear.
     */
    fun getMotorToLinearSubsystemDisplacement(reduction: Reduction, sprocket: Sprocket): Distance {
        return sprocket.angularDisplacementToLinearDisplacement(
            reduction.apply(getPosition())
        )
    }

    /**
     * Applies reduction, and then uses the [sprocket] diameter to convert from angular to linear.
     */
    fun getMotorToLinearSubsystemVelocity(reduction: Reduction, sprocket: Sprocket): LinearVelocity {
        return sprocket.angularVelocityToLinearVelocity(
            reduction.apply(getVelocity())
        )
    }

    /**
     * Just unapplies the reduction to the given measurement.
     */
    fun getAngularSubsystemToMotorPosition(measurement: Angle, reduction: Reduction): Angle {
        return reduction.unapply(measurement)
    }
    /**
     * Just unapplies the reduction to the given measurement.
     */
    fun getAngularSubsystemToMotorVelocity(measurement: AngularVelocity, reduction: Reduction): AngularVelocity {
        return reduction.unapply(measurement)
    }

    fun getAngularSubsystemToMotorAcceleration(measurement: AngularAcceleration, reduction: Reduction): AngularAcceleration {
        return reduction.unapply(measurement)
    }

    /**
     * Conversion as follows:
     *
     * Measurement (linear) -> Sprocket -> Measurement (angular) -> Reduction -> Motor
     */
    fun getLinearSubsystemToMotorPosition(measurement: Distance, reduction: Reduction, sprocket: Sprocket): Angle {
        return reduction.unapply(
            sprocket.linearDisplacementToAngularDisplacement(measurement)
        )
    }

    /**
     * Conversion as follows:
     *
     * Measurement (linear) -> Sprocket -> Measurement (angular) -> Reduction -> Motor
     */
    fun getLinearSubsystemToMotorVelocity(measurement: LinearVelocity, reduction: Reduction, sprocket: Sprocket): AngularVelocity {
        return reduction.unapply(
            sprocket.linearVelocityToAngularVelocity(measurement)
        )
    }

    // --------------- ------------- ------- --------------- //
    // --------------- CONFIGURATION METHODS --------------- //
    // --------------- ------------- ------- --------------- //

    /**
     * Sets all unnecessary signals to 4Hz.
     * Only signals written inside the method are preserved for real-time usage.
     * Consult an Area Lead / Captain if you consider another signal as necessary.
     */
    private fun optimizeMotorCan() {
        motor.optimizeBusUtilization()
        with(motor) {
            position.setUpdateFrequency(if (useHigherFrequency) HIGHER_SIGNAL_FREQUENCY else DEFAULT_SIGNAL_FREQUENCY)
            velocity.setUpdateFrequency(if (useHigherFrequency) HIGHER_SIGNAL_FREQUENCY else DEFAULT_SIGNAL_FREQUENCY)

            acceleration.setUpdateFrequency(DEFAULT_SIGNAL_FREQUENCY)
            closedLoopReference.setUpdateFrequency(DEFAULT_SIGNAL_FREQUENCY)
            torqueCurrent.setUpdateFrequency(DEFAULT_SIGNAL_FREQUENCY)   // Required by followers (Phoenix 6 documentation)
            motorVoltage.setUpdateFrequency(DEFAULT_SIGNAL_FREQUENCY)    // Required by followers (Phoenix 6 documentation)
            supplyCurrent.setUpdateFrequency(DEFAULT_SIGNAL_FREQUENCY)
            dutyCycle.setUpdateFrequency(DEFAULT_SIGNAL_FREQUENCY)       // Required by followers (Phoenix 6 documentation)

            controlMode.setUpdateFrequency(LOWER_SIGNAL_FREQUENCY)
            deviceTemp.setUpdateFrequency(LOWER_SIGNAL_FREQUENCY)
            version.setUpdateFrequency(LOWER_SIGNAL_FREQUENCY)
        }
    }

    /**
     * First, clears all sticky faults (which happen after a brownout, unlicensed pro feature access, etc.)
     * and then applies the [config].
     * Should only be called during initial configuration. There are dedicated methods to update SlotConfigs and CurrentLimits
     * @see applySlotConfigs
     * @see applyCurrentLimitConfigs
     */
    fun applyFullConfigAndClearFaults(config: TalonFXConfiguration) {
        motor.clearStickyFaults()
        tryUntilOk(MAX_APPLY_ATTEMPTS_FULL_CONFIG) { motor.configurator.apply(config, FULL_CONFIG_APPLYING_TIMEOUT.seconds) }
    }

    /**
     * Used specifically for updating Control Gain configs without reapplying everything else, saving loop cycle time.
     * An [Exception] will be thrown if commanded by a follower motor, as it does not need this (just mirrors lead).
     */
    fun applySlotConfigs(slot0Configs: Slot0Configs, slot1Configs: Slot1Configs, slot2Configs: Slot2Configs) {
        if (isFollower) { throw commandToFollowerException() }
        if (getIsConnected()) {
            tryUntilOk(MAX_APPLY_ATTEMPTS_SINGLE_CONFIG) { motor.configurator.apply(slot0Configs, SINGLE_CONFIG_APPLYING_TIMEOUT.seconds) }
            tryUntilOk(MAX_APPLY_ATTEMPTS_SINGLE_CONFIG) { motor.configurator.apply(slot1Configs, SINGLE_CONFIG_APPLYING_TIMEOUT.seconds) }
            tryUntilOk(MAX_APPLY_ATTEMPTS_SINGLE_CONFIG) { motor.configurator.apply(slot2Configs, SINGLE_CONFIG_APPLYING_TIMEOUT.seconds) }
        }
    }

    /**
     * Used specifically for updating [CurrentLimitsConfigs] without reapplying everything else, saving loop cycle time.
     * An [Exception] will be thrown if commanded by a follower motor, as it does not need this (just mirrors lead).
     */
    fun applyCurrentLimitConfigs(currentLimits: CurrentLimitsConfigs) {
        if (isFollower) { throw commandToFollowerException() }
        if (getIsConnected())
            tryUntilOk(MAX_APPLY_ATTEMPTS_SINGLE_CONFIG) { motor.configurator.apply(currentLimits, SINGLE_CONFIG_APPLYING_TIMEOUT.seconds) }
    }

    /**
     * Sends a follower request of this motor to the [leadingMotorController].
     * Keep in mind that for subsystem control, you will need to command the [leadingMotorController] and not the one
     * you are calling this method from; doing so will result in an [IllegalCallerException].
     */
    fun follow(leadingMotorController: TalonFX, opposeMaster: MotorAlignmentValue) {
        isFollower = true
        motor.setControl(Follower(leadingMotorController.deviceID, opposeMaster))
    }

    /** Changes Neutral Mode to Coast */
    fun coast() { motor.setNeutralMode(NeutralModeValue.Coast) }

    /** Changes Neutral Mode to Brake */
    fun brake() { motor.setNeutralMode(NeutralModeValue.Brake) }

    /** Just stops the motor. Must be called only by lead motor. */
    fun stopMotor() {
        if (isFollower) { throw commandToFollowerException() }
        motor.stopMotor()
    }
}
