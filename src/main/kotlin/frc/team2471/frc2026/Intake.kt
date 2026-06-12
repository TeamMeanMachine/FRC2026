package frc.team2471.frc2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
//import com.ctre.phoenix6.signals.MotorAlignmentValue
//import edu.wpi.first.wpilibj2.command.Command
//import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
//import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.commands.MechanismBase
import org.team2471.frc.lib.commands.onCancel
import org.team2471.frc.lib.commands.periodic
import org.team2471.frc.lib.commands.use
import org.team2471.frc.lib.control.CurrentLimits
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.units.seconds
import org.team2471.frc.lib.util.PowerTracker
//import org.team2471.frc.lib.control.commands.finallyRun
//import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
//import org.team2471.frc.lib.control.commands.onlyRunWhileTrue
//import org.team2471.frc.lib.control.commands.parallelCommand
//import org.team2471.frc.lib.control.commands.runCommand
//import org.team2471.frc.lib.control.commands.runOnceCommand
//import org.team2471.frc.lib.control.commands.sequenceCommand
//import org.team2471.frc.lib.control.commands.waitCommand
import org.team2471.frc.lib.util.isSim
import org.wpilib.command3.Command
import org.wpilib.hardware.discrete.DigitalInput
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.system.Timer

object Intake: MechanismBase("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")

    val deployPoseEntry = table.getEntry("deployPose")
    val stowPoseEntry = table.getEntry("stowPose")
    val deepStowPoseEntry = table.getEntry("deepStowPose")
    val intakePowerEntry = table.getEntry("intakePower")

    val maxForwardTorqueEntry = table.getEntry("maxForwardTorque")
    val maxForwardTorque get() = maxForwardTorqueEntry.getDouble(18.0)
    var prevMaxForwardTorque = maxForwardTorque

    val DEPLOY_POSE get() = deployPoseEntry.getDouble(if (Robot.isCompBot) 29.0 else 25.75)
    val STOW_POSE get() = stowPoseEntry.getDouble(if (Robot.isCompBot) 2.0 else 2.0)
    val DEEP_STOW_POSE get() = deepStowPoseEntry.getDouble(0.0)

    val INTAKE_POWER get() = intakePowerEntry.getDouble(if (Robot.isCompBot) 75.0 else 75.0)
    val HOMING_POWER = if (Robot.isCompBot) 0.1 else 0.15

    const val HOME_VELOCITY_THRESHOLD = 0.25

//    val rollerMotor = TalonFX(Falcons.INTAKE_ROLLER_0, if (Robot.isCompBot) CANivores.INTAKE_CAN else CANBus("rio")) // TODO: PHOENIX 6 2027
//    val rollerMotorFollower = TalonFX(Falcons.INTAKE_ROLLER_1, if (Robot.isCompBot) CANivores.INTAKE_CAN else CANBus("rio"))
//    val deployMotor0 = TalonFX(Falcons.INTAKE_DEPLOY_0)
//    val deployMotor1 = TalonFX(Falcons.INTAKE_DEPLOY_1)
    val stopSensor0 = DigitalInput(DigitalSensors.INTAKE_STOP_SENSOR_0)
    val stopSensor1 = DigitalInput(DigitalSensors.INTAKE_STOP_SENSOR_1)

//    @get:AutoLogOutput(key = "Intake/Intake state") TODO
    var intakeState: IntakeState = IntakeState.OFF
    var prevIntakeState = intakeState

//    @get:AutoLogOutput(key = "Intake/Hit Hard Stop 0") TODO
    val hitHardStop0 get() = !stopSensor0.get()

//    @get:AutoLogOutput(key = "Intake/Hit Hard Stop 1") TODO
    val hitHardStop1 get() = !stopSensor1.get()

//    @get:AutoLogOutput(key = "Intake/Roller Motor Temp") TODO
    val rollerTemp get() = 0.0//rollerMotor.deviceTemp.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Intake/Velocity Setpoint") TODO
    var velocitySetpoint: Double = 0.0
        set(value) {
            field = value.coerceIn(-100.0, 100.0)
            if (field == 0.0) {
//                rollerMotor.setControl(NeutralOut()) // TODO: PHOENIX 6 2027
            } else {
//                rollerMotor.setControl(DutyCycleOut(field / 100.0).withEnableFOC(true))// TODO: PHOENIX 6 2027
            }
        }

    var lastReachedSetpoint = 0.0

    const val REACHED_SETPOINT_THRESHOLD = 0.05

//    @get:AutoLogOutput(key = "Intake/Deploy Setpoint") TODO
    var deploySetpoint: Double = 0.0
        set(value) {
            field = value
            if (finishedHoming) {
                if (disableSpringProtection) {
//                    deployMotor0.setControl(MotionMagicVoltage(field).withSlot(1))// TODO: PHOENIX 6 2027
                    if (Robot.isCompBot) {
//                        deployMotor1.setControl(MotionMagicVoltage(field).withSlot(1))// TODO: PHOENIX 6 2027
                    }
                } else {
//                    deployMotor0.setControl(PositionTorqueCurrentFOC(field))// TODO: PHOENIX 6 2027
                    if (Robot.isCompBot) {
//                        deployMotor1.setControl(PositionTorqueCurrentFOC(field))// TODO: PHOENIX 6 2027
                    }
                }


//                if (lastReachedSetpoint != value) {
//                    reachedSetpoint0 = false
//                    reachedSetpoint1 = false
//                    lastReachedSetpoint = value
//                }
//
//                if (Robot.isCompBot) {
//                    if (deployMotor0Error.absoluteValue < REACHED_SETPOINT_THRESHOLD && deployVelocity0.absoluteValue < 0.2) {
//                        reachedSetpoint0 = true
//                    }
//                    if (deployMotor1Error.absoluteValue < REACHED_SETPOINT_THRESHOLD && deployVelocity1.absoluteValue < 0.2) {
//                        reachedSetpoint1 = true
//                    }
//                } else {
//                    if (deployMotor0Error.absoluteValue < 0.5) {
//                        reachedSetpoint0 = true
//                        reachedSetpoint1 = true
//                    }
//                }
//
//
//                if (reachedSetpoint0 && reachedSetpoint1) {
//                    // Relies on the optimization thing where it won't evaluate the statements after the or if the first statement returns true, to prevent accessing deployMotor1 stuff when it doesn't exist
//                    if (!Robot.isCompBot || (deployMotor0Position - deployMotor1Position).absoluteValue < FLEX_THRESHOLD) {
//                        if (deployMotor0Error < -0.7) {
//                            deployMotor0.setControl(TorqueCurrentFOC(21.0))
//                        } else {
//                            deployMotor1.setControl(NeutralOut())
////                            if (deployMotor0Error.absoluteValue < 0.3) {
////                                deployMotor0.setControl(NeutralOut())
////                            } else {
////                                deployMotor0.setControl(MotionMagicVoltage(field))
////                            }
//                        }
//
//                        if (Robot.isCompBot) {
//                            if (deployMotor1Error < -0.7) {
//                                deployMotor1.setControl(TorqueCurrentFOC(21.0))
//                            } else {
//                                deployMotor1.setControl(NeutralOut())
////                                if (deployMotor1Error.absoluteValue < 0.3) {
////                                    deployMotor1.setControl(NeutralOut())
////                                } else {
////                                    deployMotor1.setControl(MotionMagicVoltage(field))
////                                }
//                            }
//                        }
//                    } else {
//                        if (deployMotor0Position > deployMotor1Position) {
//                            deployMotor0.setControl(MotionMagicVoltage(deployMotor1Position))
//                            deployMotor1.setControl(TorqueCurrentFOC(21.0))
//                        } else {
//                            deployMotor1.setControl(MotionMagicVoltage(deployMotor0Position))
//                            deployMotor0.setControl(TorqueCurrentFOC(21.0))
//                        }
//                    }
//                } else {
//                    deployMotor0.setControl(MotionMagicVoltage(field))
//                    if (Robot.isCompBot) {
//                        deployMotor1.setControl(MotionMagicVoltage(field))
//                    }
//                }
            }
        }

//    @get:AutoLogOutput(key = "Intake/Roller Velocity") TODO
    val rollerVelocity: Double
        get() = 0.0//rollerMotor.velocity.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Intake/Roller Current") TODO
    val rollerCurrent: Double
        get() = 0.0//rollerMotor.supplyCurrent.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Intake/Deploy0 Current") TODO
    val deployCurrent0: Double
        get() = 0.0//deployMotor0.supplyCurrent.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Intake/Deploy1 Current") TODO
    val deployCurrent1: Double
        get() = 0.0//deployMotor1.supplyCurrent.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Intake/Deploy0 Velocity") TODO
    val deployVelocity0: Double
        get() = 0.0//deployMotor0.velocity.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Intake/Deploy1 Velocity") TODO
    val deployVelocity1: Double
        get() = 0.0//deployMotor1.velocity.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Intake/Deploy Motor Position") TODO
    val deployMotor0Position: Double
        get() = 0.0//deployMotor0.position.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Intake/Deploy Motor Follower Position") TODO
    val deployMotor1Position: Double
        get() = 0.0//if (Robot.isCompBot) deployMotor1.position.valueAsDouble else 0.0 //TODO: UNCOMMENT WHEN 2027 PHOENIX 6


//    @get:AutoLogOutput(key = "Intake/Deploy Motor Error") TODO
    val deployMotor0Error: Double
        get() = deployMotor0Position - deploySetpoint//deployMotor.closedLoopError.valueAsDouble

//    @get:AutoLogOutput(key = "Intake/Deploy Motor Follower Error") TODO
    val deployMotor1Error: Double
        get() = deployMotor1Position - deploySetpoint//deployMotor.closedLoopError.valueAsDouble


    var finishedHoming: Boolean = false
    var isDeployed: Boolean = false
    var disableSpringProtection = false

//    @get:AutoLogOutput(key = "Intake/goingToSetpoint0") TODO
    var goingToSetpoint0: Boolean = false

//    @get:AutoLogOutput(key = "Intake/goingToSetpoint1") TODO
    var goingToSetpoint1: Boolean = false

//    @get:AutoLogOutput(key = "Intake/reachedSetpoint0") TODO
    var reachedSetpoint0: Boolean = false
//    @get:AutoLogOutput(key = "Intake/reachedSetpoint1") TODO
    var reachedSetpoint1: Boolean = false


    const val FLEX_THRESHOLD = 3.0

    val autoCurrentLimits = CurrentLimits(20.0, 30.0, 1.0)
//    val teleopCurrentLimits = CurrentLimits(30.0, 40.0, 1.0)


    init {
        if (!deployPoseEntry.exists()) deployPoseEntry.setDouble(DEPLOY_POSE)
        if (!stowPoseEntry.exists()) stowPoseEntry.setDouble(STOW_POSE)
        if (!deepStowPoseEntry.exists()) deepStowPoseEntry.setDouble(DEEP_STOW_POSE)
        if (!intakePowerEntry.exists()) intakePowerEntry.setDouble(INTAKE_POWER)
        if (!maxForwardTorqueEntry.exists()) maxForwardTorqueEntry.setDouble(maxForwardTorque)

        deployPoseEntry.setPersistent()
        stowPoseEntry.setPersistent()
        deepStowPoseEntry.setPersistent()
        intakePowerEntry.setPersistent()
        maxForwardTorqueEntry.setPersistent()


        // Create Intake deploy motor configuration
//        val deployConfig = TalonFXConfiguration().apply { // TODO: PHOENIX 6 2027
//            currentLimits(5.0, 25.0, 0.25)
//            coastMode()
////            if (Robot.isCompBot) {
//                p(1.5, 1)
//                s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign, 1)
////
////            } else {
////                p(1.5)
////                s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign)
////            }
//            p(50.0)
//            d(3.0)
//
//            TorqueCurrent.PeakForwardTorqueCurrent = maxForwardTorque
//
//            if (Robot.isCompBot) motionMagic(200.0, 500.0) else motionMagic(750.0, 1500.0)
//        }

        // Apply config to motors
//        deployMotor0.applyConfiguration(deployConfig.apply { inverted(true) }) // TODO: PHOENIX 6 2027
//        deployMotor0.setPosition(0.0)
//
//        if (Robot.isCompBot) {
//            deployMotor1.applyConfiguration(deployConfig.apply { inverted(false) })
//            deployMotor1.setPosition(0.0)
//        }
//
//        rollerMotor.applyConfiguration {
//            currentLimits(autoCurrentLimits.peakLimit, autoCurrentLimits.continuousLimit, autoCurrentLimits.peakDuration)
////            p(7.0)
////            s(10.0, StaticFeedforwardSignValue.UseVelocitySign)
//            coastMode()
//        }
//        if (Robot.isCompBot) {
//            rollerMotor.addFollower(rollerMotorFollower/*, false*/)
//        } else {
//            rollerMotor.addFollower(rollerMotorFollower)
//        }

        if (!isSim) {
            PowerTracker.addMotors("Intake Roller", { rollerCurrent }, 2, {/*rollerMotor.supplyVoltage.value.asVolts*/0.0}) //TODO: UNCOMMENT WHEN 2027 PHOENIX 6
            PowerTracker.addMotors("Intake Deploy 0", { deployCurrent0 })
            if (Robot.isCompBot) {
                PowerTracker.addMotors("Intake Deploy 1", { deployCurrent1 })
            }
        }

//        this.defaultCommand = use("Intake Default", this) { default()}//default()//.ignoringDisable(true)

        this.defaultCommand = use("Intake Default", this) {
            periodic(0.0) {
                LoopLogger.record("b4 Intake default")
            }
        }


        GlobalScope.launch {
            org.team2471.frc.lib.coroutines.periodic {
                deploySetpoint = deploySetpoint
            }
        }
    }

    override fun periodic() {
        if (maxForwardTorque != prevMaxForwardTorque) {
            GlobalScope.launch {
//                deployMotor0.modifyConfiguration { // TODO: PHOENIX 6 2027
//                    TorqueCurrent.PeakForwardTorqueCurrent = maxForwardTorque
//                }
//                deployMotor1.modifyConfiguration {
//                    TorqueCurrent.PeakForwardTorqueCurrent = maxForwardTorque
//                }
            }
            prevMaxForwardTorque = maxForwardTorque
        }
    }

    fun deploy() {
//        if (deployMotor0Error.absoluteValue > FLEX_THRESHOLD) {
//            goingToSetpoint0 = true
//        }
//        if (Robot.isCompBot && deployMotor1Error.absoluteValue > FLEX_THRESHOLD) {
//            goingToSetpoint1 = true
//        }
        deploySetpoint = DEPLOY_POSE
        isDeployed = true
    }

    fun stow() {
//        goingToSetpoint0 = true
//        if (Robot.isCompBot) goingToSetpoint1 = true
        deploySetpoint = STOW_POSE
        isDeployed = false
    }

    fun deepStow() {
//        goingToSetpoint0 = true
//        if (Robot.isCompBot) goingToSetpoint1 = true
        deploySetpoint = DEEP_STOW_POSE
        isDeployed = false
    }

    fun home(): Command = use("Home", this) {
        finishedHoming = false
//        parallel( // TODO: PHOENIX 6 2027
//            homeMotorOut(deployMotor0, { deployVelocity0.absoluteValue < HOME_VELOCITY_THRESHOLD && deployVelocity1.absoluteValue < HOME_VELOCITY_THRESHOLD }),
//            homeMotorOut(deployMotor1, { deployVelocity1.absoluteValue < HOME_VELOCITY_THRESHOLD && deployVelocity0.absoluteValue < HOME_VELOCITY_THRESHOLD })
//        )
        finishedHoming = true
        deploySetpoint = DEPLOY_POSE
    }

//    fun home(): Command  = if (Robot.isCompBot) {
//        parallelCommand(
//            runOnceCommand {
//                finishedHoming = false
//            },
//            homeMotorOut(deployMotor0, { deployVelocity0.absoluteValue < HOME_VELOCITY_THRESHOLD && deployVelocity1.absoluteValue < HOME_VELOCITY_THRESHOLD }),
//            homeMotorOut(deployMotor1, { deployVelocity1.absoluteValue < HOME_VELOCITY_THRESHOLD && deployVelocity0.absoluteValue < HOME_VELOCITY_THRESHOLD })
//        ).finallyRun {
//            finishedHoming = true
//            deploySetpoint = DEPLOY_POSE
//        }
//    } else {
//        sequenceCommand(
//            runOnceCommand {
//                finishedHoming = false
//            },
//            homeMotor(deployMotor0, { hitHardStop0 }),
//            runOnceCommand {
//                finishedHoming = true
//                stow()
//            }
//        )
//    }.apply {
//        addRequirements(Intake, Shooter)
//    }

//    private fun homeMotor(motor: TalonFX, hitHardStopSupplier: () -> Boolean): Command {
//        return sequenceCommand(
//            runCommand {
//                println("going in?")
//                motor.setControl(DutyCycleOut(HOMING_POWER))
//            }.onlyRunWhileTrue { hitHardStopSupplier.invoke() },
//            runCommand {
//                println("going out?")
//                motor.setControl(DutyCycleOut(-HOMING_POWER))
//            }.onlyRunWhileFalse { hitHardStopSupplier.invoke() }.withTimeout(6.0).finallyRun {
//                motor.setControl(DutyCycleOut(0.0))
//                println("Deploy Pos: ${motor.position}")
//                motor.setPosition(if (Robot.isCompBot) 0.11 else 0.13)
//            }
//        )
//    }

    // V3 Commands
    private fun homeMotorOut(motor: TalonFX, hitHardStopSupplier: () -> Boolean): Command = use("HomeMotorOut") {
        val timer = Timer()
        timer.start()
        periodic {
            if ((hitHardStopSupplier.invoke() && timer.get() > 0.5) || timer.get() > 6.0) {
                stop()
            } else {
                println("going out?")
                motor.setControl(DutyCycleOut(HOMING_POWER))
            }
        }
        motor.setControl(DutyCycleOut(0.0))
//        println("Deploy Pos: ${motor.position}") //TODO: UNCOMMENT WHEN 2027 PHOENIX 6
        motor.setPosition(DEPLOY_POSE + 0.5)
    }

    fun pulse() = use(this) {
        periodic {
            if (Robot.isAutonomous || OI.driverController.rightTriggerAxis > 0.75) {
                stow()
                wait(0.25.seconds)
                deploy()
                wait(0.25.seconds)
            } else {
                stop()
            }
        }
        stow()
    }.onCancel {
        stow()
    }


    fun homeDeploy(): Command = use(this) {
//        deployMotor0.setPosition(deploySetpoint) // TODO: PHOENIX 6 2027
//        if (Robot.isCompBot) deployMotor1.setPosition(deploySetpoint)
    }


    override fun default() = defaultCommand {
        periodic {
            LoopLogger.record("b4 Intake default")
            when (intakeState) {
                IntakeState.OFF -> {
                    velocitySetpoint = 0.0
                }
                IntakeState.INTAKING -> {
                    velocitySetpoint = if (Robot.isAutonomous) 100.0 else INTAKE_POWER
                    if (!Shooter.isShooting) {
                        Spindexer.currentState = Spindexer.State.AGITATING
                    }
                }
                IntakeState.SPITTING -> {
                    velocitySetpoint = -INTAKE_POWER
                }
            }

            LoopLogger.record("Intake default when")

            if (prevIntakeState == IntakeState.INTAKING && intakeState != IntakeState.INTAKING) {
                Spindexer.currentState = Spindexer.State.OFF
            }
            prevIntakeState = intakeState

//        if (goingToSetpoint0 && deployMotor0Error.absoluteValue < FLEX_THRESHOLD) {
//            goingToSetpoint0 = false
//        }

//        if (Robot.isCompBot && goingToSetpoint1 && deployMotor1Error.absoluteValue < FLEX_THRESHOLD) {
//            goingToSetpoint1 = false
//        }


//        LoopLogger.record("Intake default b4 controlMode")


            LoopLogger.record("Intake default")
        }
    }

    enum class IntakeState {
        INTAKING,
        OFF,
        SPITTING,
    }
}