package frc.tecdroid3354.subsystems.templates.linearDisplacement;

import edu.wpi.first.units.measure.*;
import frc.tecdroid3354.utils.interfaces.MotorIO;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

/**
 * I/O (Input/Output) interface intended for any {@link Distance} driven subsystem, i.e. an Elevator.
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
public interface ElevatorIO {
    /**
     * Class intended to log all relevant fields that might change during a match.
     * These inputs may be used for:
     *  <p>- Tune ControlGains constants in AdvantageScope (using Mechanical Advantage's LoggedTunableNumber)</p>
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
    class ElevatorIOInputs {
        /** Elevator wise fields */
        public MutDistance elevatorDisplacement = Meters.mutable(0.0);          // Actual Displacement
        public MutDistance elevatorTargetDisplacement = Meters.mutable(0.0);    // Setpoint / Target
        public MutDistance elevatorManualTargetDisplacement = Meters.mutable(0.0); // Manually set live
    }

    /**
     * Intended to update any relevant fields in {@link ElevatorIOInputs ElevatorIOInputs}.
     * Might change depending on the implementation (i.e. simulation does not need to check motors' connectivity).
     * @param inputs The generated {Mechanism}IOInputsAutoLogged object keeping track of everything.
     */
    default void updateElevatorInputs(ElevatorIOInputs inputs,
                              MotorIO.MotorIOInputs leadMotorInputs, MotorIO.MotorIOInputs followerMotorInputs) {};

    /**
     * Used to update the in-file variable containing the manual target displacement. This resets with every code reload.
     * Testing / Showcase purposes.
     * @param newElevatorManualDisplacement Obtained live through Elastic.
     */
    default void updateElevatorManualDisplacement(Distance  newElevatorManualDisplacement) {};

    /**
     * Used to re-configure the motors with the new PID, SVAG gains. These gains reset with every code reload.
     * All other settings are obtained through the pre-established motor configurations.
     * The gains will be gathered from the respective {@link frc.tecdroid3354.utils.controlProfiles.TunableControlGains}
     * inside {@link frc.tecdroid3354.constants.SubsystemsControlGains}, based on the desired slot.
     * @param slot Which control gains slot to update [0, 1, 2]
     */
    default void updateElevatorMotorsControlGains(int slot) {};

    /**
     * Changes the target displacement of the subsystem to the one set
     * in {@link #updateElevatorManualDisplacement(Distance)}
     */
    default Runnable setElevatorManualTargetDisplacement() { return null; };

    /**
     * Changes the target displacement of the subsystem.
     * @param elevatorTargetDisplacement The desired displacement for the subsystem.
     */
    default Runnable setElevatorTargetDisplacement(Distance elevatorTargetDisplacement) { return null; };

    /**
     * Stops the Elevator motors. Intended for SysId, or an abrupt manual stop.
     */
    default Runnable stopElevator() { return null; };

    /**
     * Merely changes the Neutral / Idle mode of the motors to coast for easier manipulation.
     */
    default Runnable coastElevatorMotors() { return null; };

    /**
     * Merely changes the Neutral / Idle mode of the motors to brake to avoid unintended movement during match.
     */
    default Runnable brakeElevatorMotors() { return null; };

    /**
     * Applies the configuration inside {Mechanism}Constants.PhoenixMotorConfiguration. Follower commands are included.
     */
    default void initialMotorConfiguration() {};
}
