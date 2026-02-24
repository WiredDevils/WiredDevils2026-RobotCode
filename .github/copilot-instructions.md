## Quick orientation for AI coding agents

This repository is a WPILib Java robot project (GradleRIO). Focus on command-based structure: subsystems under `src/main/java/frc/robot/subsystems`, commands under `src/main/java/frc/robot/commands` (autons in `commands/AutonCommands`), and wiring in `src/main/java/frc/robot/RobotContainer.java`.

- Build & test (Windows):
  - `gradlew.bat build` — compile and assemble the fat jar
  - `gradlew.bat test` — run JUnit 5 unit tests
  - `gradlew.bat deploy` — deploy artifact(s) to the configured RoboRIO target (team comes from `.wpilib/wpilib_preferences.json` or pass via CLI)

- Simulation notes:
  - This project enables simulation GUI and DriverStation via `wpi.sim.addGui()` and `wpi.sim.addDriverstation()` in `build.gradle`.
  - Desktop support is toggled with the `includeDesktopSupport` flag in `build.gradle`.

- Key integration points and 3rd-party libs:
  - PathPlanner: `com.pathplanner.lib.*` is used for auto building and named commands (see `RobotContainer.registerNamedCommands()` and `AutoBuilder.buildAutoChooser()`).
  - Vendor dependencies are stored under `vendordeps/` and wired via `build.gradle` (Phoenix/REV/Pathplanner JSON files present).
  - Logging utility: `frc.lib.util.loggingUtil.LogManager` is used from `Robot.robotPeriodic()`.

- Typical architecture patterns to follow (concrete examples):
  - Wiring lives in `RobotContainer.java`. Default commands are set with `subsystem.setDefaultCommand(...)` (example: `s_Swerve.setDefaultCommand(new TeleopSwerve(... () -> -Math.pow(driver.getRawAxis(translationAxis),3) ...))`).
  - Named PathPlanner commands are registered with `NamedCommands.registerCommand("Name", command)` (see `registerNamedCommands()` in `RobotContainer`).
  - Button bindings use `JoystickButton`/`POVButton` and `onTrue()/whileTrue()` to schedule `InstantCommand` or higher-level commands.

- Project-conventions & gotchas:
  - SmartDashboard keys are used for runtime toggles and values (examples: `Test Mode`, `Test Mode Speed Scale`, `Auto Chooser`). Prefer these exact keys when reading/writing dashboard values.
  - The robot uses a command-based style with heavy use of Suppliers/lambdas for joystick axes — prefer preserving supplier lambdas when refactoring.
  - Swerve control uses cubed input shaping (`Math.pow(axis, 3)`) in teleop. Keep that pattern unless explicitly changing drive feel.
  - Team number and debug flags are expected to be provided by WPILib prefs or CLI; do not hardcode team numbers into `build.gradle`.

- Important files to inspect for changes and examples:
  - `build.gradle` — GradleRIO plugin, dependencies, jar/deploy config
  - `src/main/java/frc/robot/RobotContainer.java` — button bindings, default commands, auto chooser
  - `src/main/java/frc/robot/Robot.java` — lifecycle hooks, scheduler, logging calls
  - `src/main/java/frc/robot/subsystems/swerve/` — swerve implementation and odometry
  - `src/main/deploy/` — static files packaged to the RoboRIO

- When adding or changing commands:
  - Follow existing pattern: create a Command class under `commands/` and if it's intended for pathplanner register with `NamedCommands` and add to `AutoBuilder` if used in autos.
  - If adding joystick bindings, prefer `RobotContainer.configureButtonBindings()` and use `JoystickButton`/`POVButton` with `onTrue()/whileTrue()`.

If anything above is unclear or you want more examples (e.g., common command templates, how to run a local simulation), tell me which area to expand and I'll iterate.
