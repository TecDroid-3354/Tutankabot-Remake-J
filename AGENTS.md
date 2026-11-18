# Project Architecture & API Playbook (FRC AdvantageKit + Phoenix 6)

## 1. Architectural Constraints
*   **Subsystem Isolation:** Subsystems (`ElevatorSubsystem`, `Drive`, etc.) **NEVER** reference or call each other. 
*   **Superstructure:** All multi-subsystem logic, sequential safety routines, and command composition live strictly in `Superstructure.kt`.
*   **Hardware Encapsulation:** Subsystems never touch hardware directly. All hardware uses `OpTalonFX` (configured via `KrakenMotors`) hidden behind the `IO.java` interface.

## 2. The 5-File Subsystem Structure
Every motion subsystem exists in `src/main/java/frc/tecdroid3354/subsystems/<mechanism>/` using this exact structure:
1.  **`<Mechanism>Constants.kt`**: (Kotlin `object`) Static hardware IDs, `Reduction`, `Sprocket`, `MeasureLimits`, and default `TalonFXConfiguration`. No logic.
2.  **`<Mechanism>IO.java`**: (Java Interface) Contains `@AutoLog class <Mechanism>IOInputs` with mutable WPILib `Measure` objects. Defines contract methods and `Dummy<Mechanism>IO` for Replay mode. **MUST BE JAVA**.
3.  **`<Mechanism>IOTalonFX.kt`**: (Kotlin) Real hardware layer. Instantiates `OpTalonFX`. Converts WPILib units to motor requests (e.g., `linearSubsystemPositionDynamicRequest`).
4.  **`<Mechanism>IOSim.kt`**: (Kotlin) WPILib Physics Sim. Reads `TalonFXSimState` voltage, updates WPILib sim, feeds results back to `TalonFXSimState`.
5.  **`<Mechanism>Subsystem.kt`**: (Kotlin `SubsystemBase`) Constructor injects `<Mechanism>IO`. Calls `Logger.processInputs()`. Exposes `Runnable` or `Command` factories.

---

## 3. Utility API Cheat Sheet

### A. WPILib Units (via `Units.kt`)
Never use `Unit.of(value)` or `value.in(Unit)`. Use the extension properties:
*   **Creation:** `5.0.meters`, `1.5.inches`, `180.0.degrees`, `2.0.rotations`, `40.0.amps`, `12.0.volts`, `0.02.seconds`, `10.0.metersPerSecond`, `2.0.metersPerSecondSquared`.
*   **Extraction:** `distance.meters`, `angle.degrees`, `velocity.rotationsPerSecond`.
*   *Note: Excludes `Unit.zero()`, which is still valid.*

### B. Command Factories (via `CommandFactoryExtensions.kt`)
Convert `Runnable` objects directly into WPILib Commands using extension functions.
*   `myRunnable.InstantCommand(subsystem)` -> Returns `InstantCommand`
*   `myRunnable.InstantCommandIgnoreDisabled(subsystem)` -> Returns `WrapperCommand` ignoring disable.
*   `myRunnable.RunCommand(subsystem)` -> Returns `RunCommand`
*   `myRunnable.InstantCommandAfterCondition({ condition }, subsystem)` -> Returns `SequentialCommandGroup`
*   `myRunnable.InstantCommandAfterTime(2.0.seconds, subsystem)` -> Returns `SequentialCommandGroup`

### C. Math & Limits
**`MeasureLimits.kt`**
*   **Init:** `val limits = MeasureLimits(0.0.meters .. 2.0.meters)`
*   **Clamp:** `val clamped: Distance = limits.coerceIn(targetDistance) as Distance`
*   **Check:** `if (distance in limits) { ... }`

**`Reductions.kt`**
*   **Init:** `val gearRatio = Reduction(8.9285)`
*   **Usage:** `gearRatio.apply(motorRotations)` (Motor -> Mechanism), `gearRatio.unapply(mechanismRotations)` (Mechanism -> Motor). Works on `Double` and `Measure`.

**`Boundary.kt`** (AdvantageScope Virtual Boxes)
*   **Init:** `Boundary("Name", isRedHalf, topLeft, topRight, bottomLeft, bottomRight)` (Uses standard Blue-Alliance Translation2d coordinates).
*   **Check:** `myBoundary.contains(robotTranslation2d)` -> Returns `Boolean`.

### D. Hardware Tuning (`TunableControlGains.kt`)
*   **Init:** `val gains = TunableControlGains("Elevator", kP, kI, kD, kS, kV, kA, kG)` *(Note: kF is intentionally excluded).*
*   **Check Updates:** `if (gains.hadTunableUpdated()) { ... }`
*   **Apply to Phoenix 6:** `motorConfig.Slot0 = gains.updatePhoenixSlot0Configs()` (Also supports Slot1, Slot2).