@file:JvmName("Main") // set the compiled Java class name to "Main" rather than "MainKt"
package frc.team2471.frc2026

//import edu.wpi.first.hal.FRCNetComm.tInstances
//import edu.wpi.first.hal.FRCNetComm.tResourceType
import com.ctre.phoenix6.SignalLogger
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.MeanLogger
import org.littletonrobotics.junction.networktables.MeanNT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team2471.frc.lib.commands.use
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.ctre.loggedTalonFX.MasterMotor
import org.team2471.frc.lib.logging.NT4NonFMSPublisher
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.util.PowerTracker
import org.team2471.frc.lib.util.RobotMode
import org.team2471.frc.lib.util.robotMode
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Scheduler
import org.wpilib.driverstation.RobotState
import org.wpilib.driverstation.internal.DriverStationBackend
import org.wpilib.framework.RobotBase
import org.wpilib.framework.TimedRobot
import org.wpilib.system.RobotController
import org.wpilib.system.Timer
import java.net.NetworkInterface
import kotlin.time.Duration.Companion.milliseconds


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@OptIn(DelicateCoroutinesApi::class)
object Robot : TimedRobot() {
    val isCompBot = getCompBotBoolean()
    private var wasDisabled = true
    private var doEnableInitAsync = false
    var beforeFirstEnable = true
        private set
    var beforeFirstEnableAsync = true
        private set

    private var wasAutonomous = false
    private var wasTeleop = false

    val scheduler = Scheduler.getDefault()

    @get:JvmName("RobotIsEnabled")
    var isEnabled = false
        private set
    @get:JvmName("RobotIsAutonomous")
    var isAutonomous = false
        private set
    @get:JvmName("RobotIsDisabled")
    var isDisabled = true
        private set
    @get:JvmName("RobotIsAutonomousEnabled")
    var isAutonomousEnabled = false
        private set

    val powerTracker = PowerTracker()


    // Subsystems:
    // MUST define an individual variable for all subsystems inside this class or else @AutoLogOutput will not work -2025
    val drive = Drive
    val oi = OI
    val shooter = Shooter
    val intake = Intake
    val turret = Turret
    val spindexer = Spindexer
    val fieldManager = FieldManager
    val aimUtils = AimUtils

    var allSubsystems = arrayOf<Mechanism>(drive, intake, shooter, turret, spindexer, oi)

    init {
        // Tells FRC we use Kotlin
//        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin)

        // Set up data receivers & replay source
        when (robotMode) {
            RobotMode.REAL -> { // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4NonFMSPublisher()) // Only log to NT if FMS is not connected
            }
            RobotMode.SIM -> {
                MeanLogger.addDataReceiver(MeanNT4Publisher())
//                MeanLogger.addDataReceiver(WPILOGWriter())
            } // Running a physics simulator, log to NT
            RobotMode.REPLAY -> { // Replaying a log, set up replay source
//                setUseTiming(true) // false - simulate as fast as possible, true - simulate in real time (particle filter needs true)
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }


        DriverStationBackend.silenceJoystickConnectionWarning(true)

        SignalLogger.setPath("")
//        SignalLogger.start()
        SignalLogger.stop()

        // Start AdvantageKit logger
        MeanLogger.start()
        // Call all subsystems, make sure their init's run
        allSubsystems.forEach { println("activating subsystem ${it.name}") }
        println("FieldManager thinks the field is ${FieldManager.fieldDimensions.measureX.asFeet} feet big")
//        println("We see ${Autonomous.paths.size} paths and they are made on the ${if (Drive.choreoPathsStartOnRed) "red" else "blue"} side.")

        GlobalScope.launch {
            // Attempt to clear out small occasional loop overruns when periodically calling DriverStation.isEnabled()
            while (true) {
                isEnabled = RobotState.isEnabled()
                isDisabled = !isEnabled
                isAutonomous = RobotState.isAutonomous()
                isAutonomousEnabled = isAutonomous && isEnabled

                if (doEnableInitAsync) {
                    enabledInitAsync()
                    doEnableInitAsync = false
                    beforeFirstEnableAsync = false
                }

                delay(10.milliseconds)
            }
        }

        RobotController.setTimeSource { RobotController.getMonotonicTime() }
        GlobalScope.launch {
            val t = Timer()
            periodic {
                powerTracker.update(t.get())
                t.restart()
            }
        }
        println("Finished Robot init")
    }

    /** This function is called periodically during all modes.  */
    override fun robotPeriodic() {
        MeanLogger.periodicBeforeUser()
        LoopLogger.reset()
        LoopLogger.record("after LL reset")
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
//         Threads.setCurrentThreadPriority(true, 99);

        if (isEnabled) {
            if (wasDisabled) {
                enabledInit()
                doEnableInitAsync = true
                beforeFirstEnable = false
                wasDisabled = false
            }
        } else {
            wasDisabled = true
        }


        LoopLogger.record("b4 Scheduler")

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
//        println("Queued Commands: ${scheduler.queuedCommands.map { it.name() }}")
        scheduler.run()

        MeanLogger.recordOutput("Scheduler/scheduler", scheduler)

        var allCommandsRuntime = 0.0
        scheduler.runningCommands.forEach {
            val runtimeS = scheduler.lastCommandRuntimeMs(it) / 1000.0
            allCommandsRuntime += runtimeS
            if (runtimeS >= 0.0) {
                MeanLogger.recordOutput("Scheduler/CommandRuntime/${it.name()}", runtimeS)
            }
        }
        MeanLogger.recordOutput("Scheduler/CommandRuntime/ALL", allCommandsRuntime)
        MeanLogger.recordOutput("Scheduler/totalRuntime", scheduler.lastRuntimeMs() / 1000.0)


        LoopLogger.record("after Scheduler")

        powerTracker.logData()
        LoopLogger.record("after powerTracker update")

        // Return to non-RT thread priority (do not modify the first argument)
//         Threads.setCurrentThreadPriority(false, 10);
        LoopLogger.record("Robot periodic()")
        MeanLogger.periodicAfterUser(0.1.toLong(), 0.1.toLong())
    }

    fun enabledInit() {}

    /** Runs on alternate thread for loop times. Doesn't run sequential with rest of robot. Runs when robot is enabled */
    fun enabledInitAsync() {
        if (beforeFirstEnableAsync) {
            if (!isAutonomous) {
                scheduler.schedule(Intake.home())

//                Drive.localizer.trackAllTags()
            } else {
//                Intake.deployMotor0.setPosition(0.0) // TODO: PHOENIX 6 2027
//                if (isCompBot) {
//                    Intake.deployMotor1.setPosition(0.0)
//                }
                Intake.finishedHoming = true

//                if (isRedAlliance) {
//                    Drive.localizer.unTrackTags(*FieldManager.blueHubTags.map { it.ID }.toTypedArray().toIntArray())
//                } else {
//                    Drive.localizer.unTrackTags(*FieldManager.redHubTags.map { it.ID }.toTypedArray().toIntArray())
//                }
            }
        }
    }

    /** This function is called once when the robot is disabled.  */
    override fun disabledInit() {
        Drive.coastMode()
        Autonomous.autonomousCommand?.let { this.scheduler.cancel(it) }
        Autonomous.testCommand?.let { this.scheduler.cancel(it) }
//        scheduler.cancel(Autonomous.autonomousCommand) // This makes sure that the autonomous stops running when teleop starts running.
//        scheduler.cancel(Autonomous.testCommand)
    }

    override fun autonomousExit() {
        println("Was autonomous")
        Drive.setDriveCurrentLimits(TunerConstants.driveTeleCurrentLimits)
//            Intake.rollerMotor.modifyConfiguration {
//                currentLimits(
//                    Intake.teleopCurrentLimits.continuousLimit,
//                    Intake.teleopCurrentLimits.peakLimit,
//                    Intake.teleopCurrentLimits.peakDuration
//                )
//            }
    }

    override fun teleopExit() {
        println("Was teleop")
//            Drive.modules.forEach {
//                GlobalScope.launch {
//                    it.driveMotor.modifyConfiguration {
//                        CurrentLimits.apply {
//                            SupplyCurrentLimit = TunerConstants.driveAutoLowerLimit
//                            SupplyCurrentLowerLimit = TunerConstants.driveAutoLowerLimit
//                            SupplyCurrentLowerTime = TunerConstants.driveAutoLowerLimit
//                            SupplyCurrentLimitEnable = false
//                        }
//                    }
//                }
//            }
    }

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {
        if (beforeFirstEnable) {
            Autonomous.updateSelectedAuto(true)
        } else {
            Autonomous.updateSelectedAuto(false)
        }
    }

    /** This function is called once when auto is enabled.  */
    override fun autonomousInit() {
//        enabledTimer.restart()
//        println("Autonomous init $timeSinceEnabled")
//        Autonomous.setDrivePositionToAutoStartPose()
//        println("scheduling auto command $timeSinceEnabled")
        scheduler.schedule(Autonomous.autonomousCommand ?: use("NullAutoCommand") { println("THE AUTONOMOUS COMMAND IS NULL")})
        wasAutonomous = true
//        println("scheduled auto command $timeSinceEnabled")
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled.  */
    override fun teleopInit() {
        wasTeleop = true
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {}

//    /** This function is called once when test mode is enabled.  */
//    override fun testInit() {
//        scheduler.cancelAll() // Cancels all running commands at the start of test mode.
//        scheduler.schedule(Autonomous.testCommand ?: use("NullTestCommand") { println("THE TEST COMMAND IS NULL") })
//    }
//
//    /** This function is called periodically during test mode.  */
//    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {}

    /** This function is called periodically whilst in simulation.  */
    @OptIn(DelicateCoroutinesApi::class)
    override fun simulationPeriodic() {
        GlobalScope.launch {
            MasterMotor.simPeriodic()
        }
    }


    private fun getCompBotBoolean(): Boolean {
        var compBot = true
        if (robotMode == RobotMode.REAL) {
            val networkInterfaces =  NetworkInterface.getNetworkInterfaces()
            println("retrieving network interfaces")
            for (iFace in networkInterfaces) {
                println(iFace.name)
                if (iFace.name == "eth0") {
                    println("NETWORK NAME--->${iFace.name}<----")
                    var macString = ""
                    for (byteVal in iFace.hardwareAddress){
                        macString += String.format("%s", byteVal)
                    }
                    println("FORMATTED---->$macString<-----")

                    compBot = (macString == "0-128475710531")
                }
            }
        } else { println("Not real so I am compbot") }
        println("I am compbot = $compBot")
        return compBot
    }


    /** Ends the main loop in startCompetition().  */
    override fun endCompetition() {
    }
}

/**
 * Main initialization function. Do not perform any initialization here
 * other than calling `RobotBase.startRobot`. Do not modify this file
 * except to change the object passed to the `startRobot` call.
 *
 * If you change the package of this file, you must also update the
 * `ROBOT_MAIN_CLASS` variable in the gradle build file. Note that
 * this file has a `@file:JvmName` annotation so that its compiled
 * Java class name is "Main" rather than "MainKt". This is to prevent
 * any issues/confusion if this file is ever replaced with a Java class.
 * See the [Package Level Functions](https://kotlinlang.org/docs/java-to-kotlin-interop.html#package-level-functions)
 * section on the *Calling Kotlin from Java* page of the Kotlin Docs.
 *
 * If you change your main frc.team2471.frc2026.Robot object (name), change the parameter of the
 * `RobotBase.startRobot` call below to the new name. (If you use the IDE's
 * Rename * Refactoring when renaming the object, it will get changed everywhere
 * including here.)
 */
fun main() = RobotBase.startRobot(DummyRobot::class.java)

//TODO: This dummy class is the only option to keep Robot an object and startRobot to successfully run.
// Perhaps figure out a way to not do this - 2027.0.0-alpha-6
class DummyRobot : TimedRobot() {
    override fun startCompetition() = Robot.startCompetition()
    override fun endCompetition() = Robot.endCompetition()
}
