package frc.tecdroid3354.subsystems.templates.angularVelocity;

import edu.wpi.first.units.measure.*;
import frc.tecdroid3354.utils.interfaces.MotorIO;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

/**
 * I/O (Input/Output) interface intended for any {@link AngularVelocity} driven subsystem, i.e. a Flywheel.
 * <p>
 * Any changes to this or any template present in the repository must be discussed with the area Lead(s) and Captain.
 * </p>
 * <p>
 * Keep all IO files in Java for correct behaviour of {@link org.littletonrobotics.junction.AutoLog @AutoLog}, since
 * Java annotations are not native to Kotlin and workarounds impact readability and performance.
 * </p>
 * <p>
 * Note that kapt is still being used, yet only to read the {@link org.littletonrobotics.junction.AutoLog @AutoLog}
 * generated files in kotlin classes, not to create them.
 * </p>
 */
public interface FlywheelIO {
    /**
     * Class intended to log all relevant fields that might change during a match.
     * These inputs may be used for:
     *  <p>- Tune PIDF constants in AdvantageScope (using Mechanical Advantage's LoggedTunableNumber)</p>
     *  <p>- Replay a match in AdvantageScope (using Replay Mode)</p>
     *
     *  All of these fields should be published periodically through
     *  {@link org.littletonrobotics.junction.Logger Logger.processInputs(String, IOInputsAutoLogged)}
     *
     *  <p>
     *      If you are having trouble with your generated {Mechanism}IOInputsAutoLogged, it is most probably
     *      because of wrong kapt configuration. Ask an Area Lead or Captain.
     *  </p>
     */
    @AutoLog
    class FlywheelIOInputs {
        /** Flywheel wise fields */
        public MutAngularVelocity flywheelActualVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularVelocity flywheelTargetVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularVelocity flywheelManualTargetVelocity = DegreesPerSecond.mutable(0.0);
        public MutAngularVelocity flywheelPresetVelocity = DegreesPerSecond.mutable(0.0); // If applicable
    }

    /**
     * Intended to update any relevant fields in {@link FlywheelIOInputs}.
     * Might change depending on the implementation (i.e. simulation does not need to check motors' connectivity).
     * @param inputs The generated {@link FlywheelIOInputs} object keeping track of everything.
     */
    default void updateFlywheelInputs(FlywheelIOInputs inputs,
                              MotorIO.MotorIOInputs leadMotorInputs, MotorIO.MotorIOInputs followerMotorInputs) {};

    /**
     * Used to update the in-file variable containing the manual target velocity. This resets with every code reload.
     * Testing / Showcase purposes.
     * @param newFlywheelManualVelocity Obtained live through Elastic.
     */
    default void updateFlywheelManualVelocity(AngularVelocity newFlywheelManualVelocity) {};

    /**
     * Used to re-configure the motors with the new PID, SVAG gains. These gains reset with every code reload.
     * All other settings are obtained through the pre-established motor configurations.
     * The gains will be gathered from the respective {@link frc.tecdroid3354.utils.controlProfiles.TunableControlGains}
     * inside {@link frc.tecdroid3354.constants.SubsystemsControlGains}, based on the desired slot.
     * @param slot Which control gains slot to update [0, 1, 2]
     */
    default void updateFlywheelMotorsControlGains(int slot) {};

    /**
     * Sets the subsystem to the manually set velocity through Elastic. This resets with every code reload.
     * <p>Make sure to update your subsystem target velocity variable for telemetry</p>
     * @see #updateFlywheelManualVelocity(AngularVelocity)
     * @return A {@link Runnable} setting the subsystem manual target velocity
     */
    default Runnable enableFlywheelManualVelocity() { return null; };

    /**
     * Only if applicable.
     * <p>Sets the subsystem to the preset velocity stored in constants.</p>
     * <p>This does not change live, only in-code.</p>
     * <p>Make sure to update your subsystem target velocity variable for telemetry</p>
     * @return A {@link Runnable} setting the subsystem preset target velocity
     */
    default Runnable enableFlywheelPresetVelocity() { return null; };

    /**
     * Only if applicable. For distance based / state based subsystems specifically.
     * <p>Sets the calculated subsystem velocity, i.e. through interpolation / polynomial</p>
     * <p>Calculation is delegated to a separate method</p>
     * <p>Make sure to update your subsystem target velocity variable for telemetry</p>
     * @param flywheelCalculatedVelocity Flywheel velocity; must be calculated beforehand.
     * @return A {@link Runnable} setting the subsystem calculated target velocity
     */
    default Runnable enableFlywheelCalculatedVelocity(AngularVelocity flywheelCalculatedVelocity) { return null; };

    /**
     * Disables the subsystem motors.
     * @return A {@link Runnable} stopping the subsystem
     */
    default Runnable stopFlywheel() { return null; };

    /**
     * Merely changes the Neutral / Idle mode of the motors to coast for easier manipulation.
     * @return A {@link Runnable} coasting all subsystem motors
     */
    default Runnable coastFlywheelMotors() { return null; };

    /**
     * Merely changes the Neutral / Idle mode of the motors to brake to avoid unintended movement during match.
     * @return A {@link Runnable} braking all subsystem motors
     */
    default Runnable brakeFlywheelMotors() { return null; };

    /**
     * Applies the configuration inside {Mechanism}Constants.PhoenixMotorConfiguration. Follower commands are included.
     */
    default void initialMotorConfiguration() {};
}
