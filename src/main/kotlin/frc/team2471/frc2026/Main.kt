@file:JvmName("Main") // set the compiled Java class name to "Main" rather than "MainKt"
package frc.team2471.frc2026

import com.ctre.phoenix6.SignalLogger
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team2471.frc.lib.autonomous.TestOpMode
import org.team2471.frc.lib.autonomous.TestRoutine
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.control.isConnected
import org.team2471.frc.lib.ctre.loggedTalonFX.MasterMotor
import org.team2471.frc.lib.energy.BatteryLogger
import org.team2471.frc.lib.logging.NT4NonFMSPublisher
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.util.RobotType
import org.team2471.frc.lib.util.robotType
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Scheduler
import org.wpilib.driverstation.DriverStationDisplay
import org.wpilib.driverstation.RobotState
import org.wpilib.driverstation.internal.DriverStationBackend
import org.wpilib.framework.OpModeRobot
import org.wpilib.framework.RobotBase
import java.net.NetworkInterface


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 *
 * 2027-alpha: Changed robot to be a class instead of an object as wpilib does not support RobotBase as an object.
 * Most things can still function inside a companion object, although makes syntax slightly strange.
 */
@OptIn(DelicateCoroutinesApi::class)
class Robot : OpModeRobot(0.01) {
    init {
        println("Robot init")
        instance = this
        // Tells FRC we use Kotlin

        // Set up data receivers & replay source
        when (robotType) {
            RobotType.REAL -> { // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4NonFMSPublisher()) // Only log to NT if FMS is not connected
            }
            RobotType.SIM -> {
                Logger.addDataReceiver(NT4Publisher())
                Logger.addDataReceiver(WPILOGWriter())
            } // Running a physics simulator, log to NT
            RobotType.REPLAY -> { // Replaying a log, set up replay source
//                setUseTiming(true) // false - simulate as fast as possible, true - simulate in real time (particle filter needs true)
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }


        DriverStationBackend.silenceJoystickConnectionWarning(true)

        SignalLogger.setPath("")
        SignalLogger.stop()

        // Start AdvantageKit logger


        val dummyRobot = DummyRobot()
        // Call all subsystems, make sure their init's run
        allSubsystems.forEach { println("activating subsystem ${it.name}") }
        println("FieldManager thinks the field is ${FieldManager.fieldDimensions.measureX.asFeet} feet big")
        println("OI driverController isConnected: ${OI.driverController.isConnected}")
        Autonomous.registerAutoOpModes()
        println("Autonomous paths count: ${Autonomous.paths.size}")

//        println("We see ${Autonomous.paths.size} paths and they are made on the ${if (Drive.choreoPathsStartOnRed) "red" else "blue"} side.")

        println("Finished Robot init")
    }

    companion object {
        val isCompBot = getCompBotBoolean()
        val scheduler = Scheduler.getDefault()

        lateinit var instance: Robot

        val isEnabled get() = RobotState.isEnabled()
        val isDisabled get() = RobotState.isDisabled()
        val isTeleop get() = RobotState.isTeleop()
        val isTeleopEnabled get() = RobotState.isTeleopEnabled()
        val isAutonomous get() = RobotState.isAutonomous()
        val isAutonomousEnabled get() = RobotState.isAutonomousEnabled()
        val isUtility get() = RobotState.isUtility()
        val isUtilityEnabled get() = RobotState.isUtilityEnabled()
        val isDSAttached get() = RobotState.isDSAttached()
        val isFMSAttached get() = RobotState.isFMSAttached()
        val isEStopped get() = RobotState.isEStopped()

        var beforeFirstEnable = true
            private set

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

        var allSubsystems = arrayOf<Mechanism>(drive, intake, shooter, turret, spindexer)

        /**
         * Disables all defaults for all subsystems, except for the [exceptions] provided.
         *
         * Designed to be called as an init function of a [TestRoutine]/[TestOpMode].
         *
         * If this function is called inside an init of a TestOpMode/Routine, it will disable all default commands only while the OpMode is selected,
         * afterward it will re-enable them. (This "scoping" feature is a part of Commandsv3/OpModes and is documented in wpilib docs)
         */
        fun disableAllDefaultCommands(vararg exceptions: Mechanism) {
            allSubsystems.filterNot { exceptions.contains(it) }.forEach {
                it.defaultCommand = it.idle()
            }
        }
    }

    /** This function is called periodically during all modes.  */
    override fun robotPeriodic() {
        LoopLogger.reset()
        LoopLogger.record("Robot periodic()")


        LoopLogger.record("Scheduler")
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        scheduler.run()
        LoopLogger.record("Scheduler")

        Logger.recordOutput("Scheduler/scheduler", scheduler)

        var allCommandsRuntime = 0.0
        scheduler.runningCommands.forEach {
            val runtimeS = scheduler.lastCommandRuntimeMs(it) / 1000.0
            allCommandsRuntime += runtimeS
            if (runtimeS >= 0.0) {
                Logger.recordOutput("Scheduler/CommandRuntime/${it.name()}", runtimeS)
            }
        }
        Logger.recordOutput("Scheduler/CommandRuntime/ALL", allCommandsRuntime)
        Logger.recordOutput("Scheduler/totalRuntime", scheduler.lastRuntimeMs() / 1000.0)



        LoopLogger.record("BatteryLogger update")
        BatteryLogger.logData()
        LoopLogger.record("BatteryLogger update")

        LoopLogger.record("Robot periodic()")
    }

    /** This function is called once when the robot is disabled.  */
    override fun disabledInit() {
        Drive.coastMode()
        beforeFirstEnable = false
    }

    /** Function called when the robot is disabled. Similar to enableInit */
    override fun disabledExit() {
        println("Robot disabled")
    }

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {}

    /** This function is called once when the robot is first started up in sim.  */
    override fun simulationInit() {}

    /** This function is called when the driver station connects.  */
    override fun driverStationConnected() {
        println("DS connected yay! ฅ^•ﻌ•^ฅ")
    }

    /** This function is called periodically whilst in simulation.  */
    @OptIn(DelicateCoroutinesApi::class)
    override fun simulationPeriodic() {
        GlobalScope.launch {
            MasterMotor.simPeriodic()
        }
    }
}

private fun getCompBotBoolean(): Boolean {
    var compBot = true
    if (robotType == RobotType.REAL) {
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

class DummyRobot: LoggedRobot() {
    init {
        Logger.start()
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
fun main() = RobotBase.startRobot(Robot::class.java)
