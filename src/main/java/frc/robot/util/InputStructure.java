package frc.robot.util;

/*
 * InputStructure.java
 *
 * Created on: Dec 20, 2025
 * Author: Chris
 *
 * What is this?
 *   This is our controls file which keeps RobotContainer.java clean. 
 *   This file allows us to have a state machine which allows us to have 
 *   progressivly more complicated mechanisms and controls.
 *
 * Changes:
 *  12-20-25 Base file created to build off of during the season
 */

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * RobotControlBindings - Centralized control bindings management for robot operation.
 * <p>
 * This class manages all control input bindings for different operator configurations.
 * Each binding method contains its own speed constants to allow for easy customization
 * when creating driver-specific control schemes.
 * <p>
 * Available control schemes:
 * - Single operator with Xbox controller
 * - Dual operator with two Xbox controllers
 * - Single operator with Logitech Extreme 3D Pro joystick
 * - Dual operator with two Logitech Extreme 3D Pro joysticks
 * - Mixed configurations (stick + Xbox)
 * - Test mode bindings
 * <p>
 * The class uses a mode-based binding system where only the selected control scheme
 * is active at any given time, preventing control conflicts between different input devices.
 */
@SuppressWarnings("UnusedReturnValue")
public class InputStructure {

    // Control chooser for dashboard
    private static final SendableChooser<BindingType> controlChooser = new SendableChooser<>();
    // Control input devices
    private final CommandXboxController driverXbox;
    private final CommandXboxController operatorXbox;
    // Robot subsystems
    private final SwerveSubsystem swerve;
    private final ControlStructure structure;

    private StringPublisher inputOverride;

    /**
     * Constructs a new RobotControlBindings instance.
     *
     * @param swerve        Swerve drivetrain subsystem
     * @param structure     Control structure for auto commands
     * 
     */
    public InputStructure(
            SwerveSubsystem swerve,
            ControlStructure structure) {

        this.driverXbox = new CommandXboxController(0);
        this.operatorXbox = new CommandXboxController(1);

        this.swerve = swerve;
        this.structure = structure;
    }
    /**
     * Initializes all control bindings and the control chooser.
     * This method should be called once during robot initialization.
     */
    public void init() {
        // Setup code side chooser swapping
        String chooserPath = "RobotTelemetry/Control Chooser";
        SmartDashboard.putData(chooserPath, controlChooser);

        this.inputOverride = NetworkTableInstance.getDefault()
                .getTable("SmartDashboard/" + chooserPath)
                .getStringTopic("selected")
                .publish();

        // Initialize all standard binding configurations
        singleXboxBindings();
        dualXboxBindings();
        testBindings();

        // Add custom driver bindings here
        // Example: xAndYBindings();

        // Log initialization
        System.out.println("Robot control bindings initialized successfully");
    }

    /**
     * Single Xbox controller bindings for solo operation.
     * All robot controls mapped to one controller with modifier buttons for speed control.
     * <p>
     * Drive Controls:
     * - Left Stick: Translation (forward/back, left/right)
     * - Right Stick X: Rotation
     * - Left Trigger (Hold): Slow mode (30% translation, 20% rotation)
     * - Right Trigger (Hold): Boost mode (100% translation, 75% rotation)
     * <p>
     * Pose Selection:
     * - D-Pad Cardinal Directions: Select reef sides (N, NE, SE, S, SW, NW)
     * - D-Pad Left/Right: Select left/right pose variants
     * - Left/Right Bumpers: Cycle station slots
     * <p>
     * Auto Commands:
     * - A Button: Auto collect from selected station
     * - B Button: Auto score L2
     * - X Button: Auto score L3
     * - Y Button: Auto score L4
     * <p>
     * Settings:
     * - Back: Toggle inverted controls
     * - Right Stick Button: Toggle field/robot relative drive
     */
    private void singleXboxBindings() {
        // Mode trigger - only active when this binding mode is selected
        Trigger isMode = BindingType.SINGLE_XBOX.isMode;
        new ControlStream(
                () -> -1 * driverXbox.getLeftX(),
                () -> -1 * driverXbox.getLeftY(),
                () -> -1 * driverXbox.getRightX(),
                isMode)
                /* Drive Speed Changing */
                .withSlowTranslation(driverXbox.leftTrigger())
                .withBoostTranslation(driverXbox.rightTrigger())
                /* Drive Type Toggles */
                .withHeadingOffset(driverXbox.back())
                .withToggleCentricity(driverXbox.start());
    }

    /**
     * Dual Xbox controller bindings for driver-operator configuration.
     * Split responsibilities between driver and operator for maximum efficiency.
     * <p>
     * Driver Controller:
     * - Left Stick: Translation
     * - Right Stick X: Rotation
     * - Left Trigger (Hold): Slow mode
     * - Right Trigger (Hold): Boost mode
     * - A Button: Auto collect from selected station
     * - B Button: Auto score L2
     * - X Button: Auto score L3
     * - Y Button: Auto score L4
     * - Back: Toggle inverted controls
     * - Right Stick Button: Toggle field/robot relative
     * <p>
     * Operator Controller:
     * - D-Pad: Pose selection (reef sides and left/right)
     * - Left/Right Bumpers: Cycle station slots
     * - Left Stick Y (Hold): Manual elevator control
     * - Right Stick X (Hold): Manual arm control (twist motion)
     * - A Button: Manual collect position
     * - B Button: Manual score L2 position
     * - X Button: Manual score L3 position
     * - Y Button: Manual score L4 position
     * - Left Trigger: Intake
     * - Right Trigger: Shoot (manual override)
     */
    private void dualXboxBindings() {
        Trigger isMode = BindingType.DUAL_XBOX.isMode;

        // Configure driver Xbox controls using helper method
        typicalDriverXboxControls(isMode);
        // Configure operator Xbox controls using helper method
        typicalOperatorXboxControls(isMode);
    }


    /**
     * Test mode bindings for debugging and system identification.
     * Provides direct control over subsystems for testing and calibration.
     * <p>
     * WARNING: Test mode bypasses safety interlocks. Use with caution.
     */
    private void testBindings() {
        /*Add test bindings as needed */
    }

    /**  Define Student Binding Methods here  */

    

    /**
     * Helper method to configure typical driver Xbox controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private ControlStream typicalDriverXboxControls(Trigger isMode) {
        return new ControlStream(
                () -> -1 * driverXbox.getLeftX(),
                () -> -1 * driverXbox.getLeftY(),
                () -> -1 * driverXbox.getRightX(),
                isMode)
                /* Drive Speed Changing */
                .withSlowTranslation(driverXbox.leftTrigger())
                .withBoostTranslation(driverXbox.rightTrigger())
                /* Drive Type Toggles */
                .withHeadingOffset(driverXbox.back())
                .withToggleCentricity(driverXbox.start());
    }

    /**
     * Helper method to configure typical operator Xbox controls.
     * Extracted for reuse across multiple binding configurations.
     *
     * @param isMode The mode trigger to gate these bindings
     */
    private ControlStream typicalOperatorXboxControls(Trigger isMode) {
        return new ControlStream(isMode);
                
    }


    /**
     * Gets the currently selected binding type.
     *
     * @return The active BindingType
     */
    public BindingType getCurrentBindingType() {
        return controlChooser.getSelected();
    }

    /**
     * Gets the control chooser for external access if needed.
     *
     * @return The SendableChooser for control selection
     */
    public SendableChooser<BindingType> getControlChooser() {
        return controlChooser;
    }

    // Control binding type enum
    public enum BindingType {
        /*
        All controls using a single xbox controller.
         */
        SINGLE_XBOX("Single Xbox", true),
        /*
        Classic driver and operator setup on two xbox controllers.
         */
        DUAL_XBOX("Dual Xbox"),
        /*
        Control mode used for testing controls subject to constant change
         */
        TESTING("Testing"),

        /**  Define Student BindingTypes here  */

        NEWSTUDENTBINDINGTYPE("New Student Binding Type");

        // BindType Name
        public final String name;
        public final Trigger isMode;

        BindingType(String name, boolean isDefault) {
            this.name = name;
            this.isMode = new Trigger(() -> controlChooser.getSelected() == this);
            if (isDefault) {
                controlChooser.setDefaultOption(name, this);
            } else {
                controlChooser.addOption(name, this);
            }
        }

        /**
         * Constructor for BindingType
         * @param name The name of the control type, used in publishing.
         */
        BindingType(String name) {
            this(name, false);
        }
    }


    // Warning suppression to clean up, suppressions are as follows.
    // {Optionals always give a warning, Some methods warn when all parts are not used, this is for when a method doesn't chain into another}
    @SuppressWarnings({"OptionalUsedAsFieldOrParameterType", "SameParameterValue", "UnusedReturnValue"})
    private class ControlStream {
        /* Trigger used to enable all controls in this Control Stream class */
        protected Optional<Trigger> isMode;
        /* Drive train control constants */
        protected Optional<Double> SLOW_TRANSLATION;
        protected Optional<Double> SLOW_ROTATION;
        protected Optional<Double> NORMAL_TRANSLATION;
        protected Optional<Double> NORMAL_ROTATION;
        protected Optional<Double> BOOST_TRANSLATION;
        protected Optional<Double> BOOST_ROTATION;
        protected Optional<SwerveInputStream> inputStream;
        protected Optional<Command> driveCommand;



        /**
         * Input stream used to streamline the construction of custom input profiles.
         *
         * @param isMode {@link Trigger} used to determine when input should be allowed.
         */
        public ControlStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, Trigger isMode) {
            /* Trigger used to enable all controls in this Control Stream class */
            this.isMode = Optional.of(isMode);
            /* Drive train control constants */
            this.SLOW_TRANSLATION = Optional.of(0.3);
            this.SLOW_ROTATION = Optional.of(0.2);
            this.NORMAL_TRANSLATION = Optional.of(0.8);
            this.NORMAL_ROTATION = Optional.of(0.6);
            this.BOOST_TRANSLATION = Optional.of(1.0);
            this.BOOST_ROTATION = Optional.of(0.75);
            this.inputStream = Optional.of(SwerveInputStream.of(
                            swerve.getSwerveDrive(),
                            x, y)
                    .cubeTranslationControllerAxis(true)
                    .withControllerRotationAxis(rotation)
                    .deadband(Constants.MiscConstants.DEADBAND)
                    .scaleTranslation(NORMAL_TRANSLATION.get())
                    .scaleRotation(NORMAL_ROTATION.get())
                    .robotRelative(false)
                    .allianceRelativeControl(true)
                    .translationHeadingOffset(Rotation2d.k180deg));
            updateDriveCommand();

            // Set default drive command when enabled
            if (driveCommand.isPresent()) {
                isMode.and(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> swerve.setDefaultCommand(getDriveCommand())));
            }
        }

        /**
         * Input stream used to streamline the construction of custom input profiles.
         *
         * @param isMode {@link Trigger} used to determine when input should be allowed.
         */
        public ControlStream(Trigger isMode) {
            /* Trigger used to enable all controls in this Control Stream class */
            this.isMode = Optional.of(isMode);
            /* Drive train control constants */
            this.SLOW_TRANSLATION = Optional.of(0.3);
            this.SLOW_ROTATION = Optional.of(0.2);
            this.NORMAL_TRANSLATION = Optional.of(0.8);
            this.NORMAL_ROTATION = Optional.of(0.6);
            this.BOOST_TRANSLATION = Optional.of(1.0);
            this.BOOST_ROTATION = Optional.of(0.75);
            /* Drive Controls not defined, I'll still define default drive variables for adding input streams after init. */
        }

        ControlStream withChangeInput(BindingType bindingType, Trigger changeInput) {
            if (isMode.isPresent()) {
                isMode.get().and(changeInput).onTrue(Commands.runOnce(() -> {
                    inputOverride.set(bindingType.name);
                }));
            } else {
                DriverStation.reportWarning("isMode not found, Change Input failed.", true);
            }

            return this;
        }

        /*  Operator Type Controls  */

        /**
         * Slows drive train speed.
         *
         * @param shouldSlow button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withSlowTranslation(Trigger shouldSlow) {
            if (isMode.isPresent() && inputStream.isPresent()
                    && SLOW_TRANSLATION.isPresent() && NORMAL_TRANSLATION.isPresent()) {
                isMode.get().and(shouldSlow).whileTrue(Commands.runEnd(
                        () -> inputStream.get().scaleTranslation(SLOW_TRANSLATION.get()),
                        () -> inputStream.get().scaleTranslation(NORMAL_TRANSLATION.get())));
            } else {
                DriverStation.reportWarning("Something not found, Slow Translation failed.", true);
            }
            return this;
        }

        /**
         * Boosts drive train speed.
         *
         * @param shouldBoost button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withBoostTranslation(Trigger shouldBoost) {
            if (isMode.isPresent() && inputStream.isPresent()
                    && BOOST_TRANSLATION.isPresent() && NORMAL_TRANSLATION.isPresent()) {
                isMode.get().and(shouldBoost).whileTrue(Commands.runEnd(
                        () -> inputStream.get().scaleTranslation(BOOST_TRANSLATION.get()),
                        () -> inputStream.get().scaleTranslation(NORMAL_TRANSLATION.get())));
            } else {
                DriverStation.reportWarning("Something not found, Boost Translation failed.", true);
            }
            return this;
        }

        /**
         * Method to switch turning the heading offset on and off.
         * Heading offset can be set with setHeadingOffset(Angle)
         * {Default : 180 degrees}
         *
         * @param shouldOffset button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withHeadingOffset(Trigger shouldOffset) {
            if (isMode.isPresent() && inputStream.isPresent()
                    && BOOST_TRANSLATION.isPresent() && NORMAL_TRANSLATION.isPresent()) {
                isMode.get().and(shouldOffset).whileTrue(Commands.runEnd(
                        () -> inputStream.get().scaleTranslation(BOOST_TRANSLATION.get()),
                        () -> inputStream.get().scaleTranslation(NORMAL_TRANSLATION.get())));
            } else {
                DriverStation.reportWarning("Something not found, Heading offset failed.", true);
            }
            return this;
        }

        /**
         * Method to switch between field and robot centric drives.
         *
         * @param shouldToggleCentricity button mapping {@link Trigger} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream withToggleCentricity(Trigger shouldToggleCentricity) {
            if (isMode.isPresent() && inputStream.isPresent()) {
                isMode.get().and(shouldToggleCentricity).toggleOnTrue(Commands.runEnd(
                        () -> inputStream.get().robotRelative(false).allianceRelativeControl(true),
                        () -> inputStream.get().robotRelative(true).allianceRelativeControl(false)
                ));
            } else {
                DriverStation.reportWarning("Something not found, Toggle Centricity failed.", true);
            }
            return this;
        }

    

        /**
         * Updates the driveCommand from the latest {@link SwerveInputStream}.
         *
         * @return {@link InputStructure} for chaining.
         */
        ControlStream updateDriveCommand() {
            if (inputStream.isPresent()) {
                this.driveCommand = Optional.of(swerve.driveFieldOriented(inputStream.get()));
                swerve.setDefaultCommand(driveCommand.get());
            } else {
                DriverStation.reportWarning("Input stream not found, updateDriveCommand failed.", false);
            }
            return this;
        }

        /**
         * Method to set the heading offset.
         * Offset is then enabled by using withHeadingOffset(Trigger)
         *
         * @param headingOffset heading offset angle.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setHeadingOffset(Angle headingOffset) {
            if (inputStream.isPresent()) {
                inputStream.get().translationHeadingOffset(new Rotation2d(headingOffset));
            } else {
                DriverStation.reportWarning("Input Stream not found, setting Heading Offset failed.", true);
            }
            return this;
        }

        

        /**
         * Gets the drive command.
         *
         * @return the {@link SwerveInputStream} or if not found will report a warning and return null.
         */
        Command getDriveCommand() {
            Command command = driveCommand.orElse(null);
            if (command == null) {
                DriverStation.reportWarning("Drive Command is null", false);
                return null;
            } else {
                return command;
            }
        }

        /**
         * Changes the default drive command.
         * This just sets the commands as the SwerveSubsystem default command.
         *
         * @param driveCommand {@link Command} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setDriveCommand(Command driveCommand) {
            this.driveCommand = Optional.of(driveCommand);
            updateDriveCommand();
            return this;
        }

        /* Control Constant Getting Methods */

        /**
         * Gets the input stream.
         *
         * @return the {@link SwerveInputStream} or if not found will report a warning and return null.
         */
        SwerveInputStream getInputStream() {
            SwerveInputStream stream = inputStream.orElse(null);
            if (stream == null) {
                DriverStation.reportWarning("Input stream is null", false);
                return null;
            } else {
                return stream;
            }
        }

        /**
         * Changes the {@link ControlStream}'s {@link SwerveInputStream} for controlling the drive train.
         *
         * @param inputStream {@link SwerveInputStream} to use.
         * @return {@link InputStructure} for chaining.
         */
        ControlStream setInputStream(SwerveInputStream inputStream) {
            this.inputStream = Optional.of(inputStream);
            return this;
        }
    }
}