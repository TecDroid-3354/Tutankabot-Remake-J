package frc.tecdroid3354.subsystems.templates.angularPosition;

import edu.wpi.first.units.measure.*;
import frc.tecdroid3354.utils.interfaces.MotorIO;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

/**
 * I/O (Input/Output) interface intended for any {@link Angle} driven subsystem, i.e. a Joint.
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
public interface JointIO {
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
    class JointIOInputs {
        /** Joint wise fields */
        public MutAngle jointActualPosition = Degrees.mutable(0.0);
        public MutAngle jointTargetPosition = Degrees.mutable(0.0);
        public MutAngle jointManualTargetPosition = Degrees.mutable(0.0);
    }

    /**
     * Intended to update any relevant fields in {@link JointIOInputs}.
     * Might change depending on the implementation (i.e. simulation does not need to check motors' connectivity).
     * @param inputs The generated {@link JointIOInputs} object keeping track of everything.
     */
    default void updateJointInputs(JointIOInputs inputs,
                           MotorIO.MotorIOInputs leadMotorInputs, MotorIO.MotorIOInputs followerMotorInputs) {};

    /**
     * Used to update the in-file variable containing the manual target position. This resets with every code reload.
     * Testing / Showcase purposes.
     * @param newJointPosition Obtained live through Elastic.
     */
    default void updateJointManualPosition(Angle newJointPosition) {};

    /**
     * Used to re-configure the motors with the new PID, SVAG gains. These gains reset with every code reload.
     * All other settings are obtained through the pre-established motor configurations.
     * The gains will be gathered from the respective {@link frc.tecdroid3354.utils.controlProfiles.TunableControlGains}
     * inside {@link frc.tecdroid3354.constants.SubsystemsControlGains}, based on the desired slot.
     * @param slot Which control gains slot to update [0, 1, 2]
     */
    default void updateJointMotorsControlGains(int slot) {};

    /**
     * Sets the subsystem to the manually set position through Elastic. This resets with every code reload.
     * <p>Make sure to update your subsystem target position variable for telemetry</p>
     * @see #updateJointManualPosition(Angle)
     * @return A {@link Runnable} setting the subsystem manual target position
     */
    default Runnable setJointManualPosition() { return null; };

    /**
     * Sets the subsystem to the given position.
     * <p>Make sure to update your subsystem target position variable telemetry</p>
     * @param jointPosition The desired position in subsystem units
     * @return A {@link Runnable} setting the subsystem to the target position
     */
    default Runnable setJointPosition(Angle jointPosition) { return null; };

    /**
     * Disables the subsystem motors.
     * @return A {@link Runnable} stopping the subsystem
     */
    default Runnable stopJoint() { return null; };

    /**
     * Merely changes the Neutral / Idle mode of the motors to coast for easier manipulation.
     * @return A {@link Runnable} coasting all subsystem motors
     */
    default Runnable coastJointMotors() { return null; };

    /**
     * Merely changes the Neutral / Idle mode of the motors to brake to avoid unintended movement during match.
     * @return A {@link Runnable} braking all subsystem motors
     */
    default Runnable brakeJointMotors() { return null; };

    /**
     * Applies the configuration inside {Mechanism}Constants.PhoenixMotorConfiguration. Follower commands are included.
     */
    default void initialMotorConfiguration() {};
}

