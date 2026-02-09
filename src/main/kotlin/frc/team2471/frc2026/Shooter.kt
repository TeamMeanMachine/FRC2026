package frc.team2471.frc2026

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.loggedTalonFX.LoggedTalonFX
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.ctre.v
import org.team2471.frc.lib.units.absoluteValue
import org.team2471.frc.lib.units.asInches
import org.team2471.frc.lib.units.asInchesPerSecond
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asMetersPerSecond
import org.team2471.frc.lib.units.asRadiansPerSecond
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.units.cos
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.inchesPerSecond
import org.team2471.frc.lib.units.metersPerSecond
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.units.seconds
import org.team2471.frc.lib.units.sin
import org.team2471.frc.lib.units.volts
import org.team2471.frc.lib.units.voltsPerSecond
import org.team2471.frc.lib.util.isReal
import org.team2471.frc.lib.util.isSim
import kotlin.math.abs
import kotlin.math.cos

// Unless otherwise specified every double here is in meters
object Shooter: SubsystemBase("Shooter") {

    val hubSpeedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(0.0, 5.975000855263158)
        put(0.30479999999999996, 5.98360885059386)
        put(0.6095999999999999, 6.0093588548558365)
        put(0.9144, 6.052032066090093)
        put(1.2191999999999998, 6.111273985393765)
        put(1.524, 6.338511713101029)
        put(1.8288, 6.41490842337587)
        put(2.1336, 6.540247994276543)
        put(2.4383999999999997, 6.679989104421275)
        put(2.7432, 6.833315345221642)
        put(3.048, 6.999389952263551)
        put(3.3528, 7.143837998151528)
        put(3.6576, 7.325573227991944)
        put(3.9623999999999997, 7.52369774940966)
        put(4.2672, 7.718260854329522)
        put(4.572, 7.979458036071344)
        put(4.876799999999999, 8.30227769050815)
        put(5.1815999999999995, 8.537510273784207)
        put(5.4864, 8.779471666768007)
        put(5.7912, 9.027635189380714)
        put(6.096, 9.281516216960513)
        put(6.400799999999999, 9.557221618512205)
        put(6.7056, 9.713394426870112)
        put(7.0104, 10.00147356268994)
        put(7.3152, 10.376595582896961)
        put(7.62, 10.512635229548158)
        put(7.924799999999999, 10.82920364199341)
        put(8.2296, 11.22326600546104)
        put(8.5344, 11.428149642589604)
        put(8.839199999999998, 11.733139324457994)
        put(9.144, 12.063781651692123)
        put(9.448799999999999, 11.791116974441833)
        put(9.753599999999999, 12.686110712469285)
        put(10.058399999999999, 13.029460090061214)
        put(10.363199999999999, 13.274947451484703)
        put(10.668, 13.595112910955907)
        put(10.9728, 13.917233756545247)
        put(11.277599999999998, 14.241188867795625)
        put(11.5824, 14.533723468537932)
        put(11.887199999999998, 14.960445780410867)
        put(12.192, 15.165509640676033)
    }
    val hubAngleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(0.0, 90.0)
        put(0.30479999999999996, 86.92631666870594)
        put(0.6095999999999999, 83.87022359406497)
        put(0.9144, 80.84871442203033)
        put(1.2191999999999998, 77.8776392771635)
        put(1.524, 74.18251478502259)
        put(1.8288, 71.51425009768978)
        put(2.1336, 68.77692608199607)
        put(2.4383999999999997, 66.14810738162157)
        put(2.7432, 63.631740389005046)
        put(3.048, 61.229835534132484)
        put(3.3528, 59.0094801241263)
        put(3.6576, 56.835776657078426)
        put(3.9623999999999997, 54.76177592309224)
        put(4.2672, 52.809745425653105)
        put(4.572, 50.91220299827429)
        put(4.876799999999999, 49.11008737866478)
        put(5.1815999999999995, 47.47164573549949)
        put(5.4864, 45.922246172345474)
        put(5.7912, 44.45697173409083)
        put(6.096, 43.07097873347284)
        put(6.400799999999999, 41.765173492867845)
        put(6.7056, 40.47595307797732)
        put(7.0104, 39.30176182864191)
        put(7.3152, 38.24850198566654)
        put(7.62, 37.08969547192753)
        put(7.924799999999999, 36.10933725783022)
        put(8.2296, 35.25071579649663)
        put(8.5344, 34.2716294512995)
        put(8.839199999999998, 33.42502222210868)
        put(9.144, 32.645681796886535)
        put(9.448799999999999, 31.222920898267517)
        put(9.753599999999999, 31.159058328298464)
        put(10.058399999999999, 30.5026441910638)
        put(10.363199999999999, 29.763965297691684)
        put(10.668, 29.13958017377846)
        put(10.9728, 28.544498959702718)
        put(11.277599999999998, 27.97689350127402)
        put(11.5824, 27.393705620324376)
        put(11.887199999999998, 27.00032258478542)
        put(12.192, 26.349602358918297)
    }

    val floorSpeedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(0.0, 4.503324999999999)
        put(0.30479999999999996, 4.513628152121638)
        put(0.6095999999999999, 4.544397453527255)
        put(0.9144, 4.595221802658169)
        put(1.2191999999999998, 4.665445819600201)
        put(1.524, 4.754209929696521)
        put(1.8288, 4.860498482216098)
        put(2.1336, 4.983190244775428)
        put(2.4383999999999997, 5.121106385892114)
        put(2.7432, 5.367987106614416)
        put(3.048, 5.638095296324615)
        put(3.3528, 5.779632870117429)
        put(3.6576, 5.9779400578056086)
        put(3.9623999999999997, 6.193987863668543)
        put(4.2672, 6.419096464142828)
        put(4.572, 6.652371399023657)
        put(4.876799999999999, 6.89300554597929)
        put(5.1815999999999995, 7.140274164072081)
        put(5.4864, 7.415277273068667)
        put(5.7912, 7.706641218414777)
        put(6.096, 7.92489864941214)
        put(6.400799999999999, 8.203553592382436)
        put(6.7056, 8.474046729467089)
        put(7.0104, 8.748162862115477)
        put(7.3152, 9.025582640553022)
        put(7.62, 9.306020639998383)
        put(7.924799999999999, 9.69035560040526)
        put(8.2296, 9.92194864373934)
        put(8.5344, 10.163019428635886)
        put(8.839199999999998, 10.453226556832286)
        put(9.144, 10.745411310923698)
        put(9.448799999999999, 11.039423741882368)
        put(9.753599999999999, 11.411260219035217)
        put(10.058399999999999, 11.724555429126072)
        put(10.363199999999999, 12.018162944534595)
        put(10.668, 12.345706521392236)
        put(10.9728, 12.711488522841673)
        put(11.277599999999998, 12.901928853766988)
        put(11.5824, 13.26344763299094)
        put(11.887199999999998, 13.42049139357928)
        put(12.192, 13.862489528017624)
    }
    val floorAngleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(0.0, 90.0)
        put(0.30479999999999996, 86.12793673212624)
        put(0.6095999999999999, 82.29092198397338)
        put(0.9144, 78.52213733978648)
        put(1.2191999999999998, 74.85127234665607)
        put(1.524, 71.30332663051043)
        put(1.8288, 67.89793201913149)
        put(2.1336, 64.64918912209924)
        put(2.4383999999999997, 61.56593551990787)
        put(2.7432, 58.40632634082576)
        put(3.048, 55.516629128413356)
        put(3.3528, 53.09185565130496)
        put(3.6576, 50.741327045839014)
        put(3.9623999999999997, 48.540387040894345)
        put(4.2672, 46.49047496767532)
        put(4.572, 44.581752193457575)
        put(4.876799999999999, 42.804331128066835)
        put(5.1815999999999995, 41.14853748759332)
        put(5.4864, 39.62095663237641)
        put(5.7912, 38.21371465655106)
        put(6.096, 36.83015595860911)
        put(6.400799999999999, 35.58680278952509)
        put(6.7056, 34.410790812804294)
        put(7.0104, 33.30833735615431)
        put(7.3152, 32.27360611862122)
        put(7.62, 31.301258860780106)
        put(7.924799999999999, 30.542276649188913)
        put(8.2296, 29.599759445218776)
        put(8.5344, 28.711869042700314)
        put(8.839199999999998, 27.944401251343578)
        put(9.144, 27.21886622514796)
        put(9.448799999999999, 26.5321868095657)
        put(9.753599999999999, 26.014007309415668)
        put(10.058399999999999, 25.425879470147827)
        put(10.363199999999999, 24.83196273623145)
        put(10.668, 24.32387742509221)
        put(10.9728, 23.907177490847104)
        put(11.277599999999998, 23.20586183437885)
        put(11.5824, 22.82710295865108)
        put(11.887199999999998, 22.10485015051786)
        put(12.192, 21.909233406770593)
    }


    val shooterMotor = LoggedTalonFX(Falcons.SHOOTER_0)
    val hoodMotor = LoggedTalonFX(Falcons.SHOOTER_HOOD)

    val WHEEL_DIAMETER = 4.0.inches

    @get:AutoLogOutput(key = "Shooter/Shooter Setpoint")
    var shooterVelocitySetpoint: LinearVelocity = 0.0.inchesPerSecond
        set(value) {
            field = value
            shooterMotor.setControl(VelocityVoltage(2.0 * field.asInchesPerSecond/(WHEEL_DIAMETER.asInches * Math.PI)))
        }


    @get:AutoLogOutput(key = "Shooter/Shooter Velocity")
    val shooterVelocity: LinearVelocity
        get() = (shooterMotor.velocity.valueAsDouble * WHEEL_DIAMETER.asInches * Math.PI).inchesPerSecond / 2.0

    @get:AutoLogOutput(key = "Shooter/Shooter Current")
    val shooterCurrent: Double
        get() = shooterMotor.supplyCurrent.valueAsDouble


    // ball trajectory angle
    @get:AutoLogOutput(key = "Shooter/Hood Angle Setpoint")
    var hoodAngleSetpoint: Angle = HOOD_STOW_SETPOINT.degrees
        set(value) {
            field = value.coerceIn(0.0.degrees, 90.0.degrees)
            hoodMotor.setControl(PositionVoltage(field))
        }

    @get:AutoLogOutput(key = "Shooter/Hood Angle")
    val hoodAngle: Angle get() = hoodMotor.position.valueAsDouble.rotations

    // degrees
    const val HOOD_STOW_SETPOINT = 90.0


    @get:AutoLogOutput(key = "Shooter/Hood error distance")
    val hoodErrorDistance get() = abs(AimUtils.distanceToGoal * sin(hoodMotor.closedLoopError.valueAsDouble.radians))

    @get:AutoLogOutput(key = "Shooter/Velocity error distance")
    val velocityErrorDistance get() = abs((if (AimUtils.aimingAtGoal) AimUtils.SHOT_AIRTIME * cos(hubAngleCurve.get(AimUtils.distanceToGoal)) else AimUtils.PASS_AIRTIME * cos(floorAngleCurve.get(AimUtils.distanceToGoal))) * shooterMotor.closedLoopError.valueAsDouble * WHEEL_DIAMETER.asMeters * Math.PI * 0.5)


    var fuel: MutableList<FuelSim> = mutableListOf()

    @get:AutoLogOutput(key = "Shooter/Ramped up")
    val rampedUp: Boolean get() = (shooterVelocity - shooterVelocitySetpoint).absoluteValue() < 1.0.metersPerSecond

    var isShooting = false
    var i = 0

    init {
        shooterMotor.configSim(DCMotor.getKrakenX60(2), 0.1)
        hoodMotor.configSim(DCMotor.getKrakenX60(1), 0.005)

        shooterMotor.applyConfiguration {
            currentLimits(25.0, 30.0, 1.0)
            coastMode()

            Feedback.withSensorToMechanismRatio(1.0/1.5)

            v(0.08)
            p(if (isReal) 0.0 else 2000.0)
            d(0.0)
            s(0.0, StaticFeedforwardSignValue.UseVelocitySign)
        }
        shooterMotor.addFollower(Falcons.SHOOTER_1)

        hoodMotor.applyConfiguration {
            currentLimits(25.0, 30.0, 1.0)
            inverted(true)
            brakeMode()
            s(0.13, StaticFeedforwardSignValue.UseClosedLoopSign)
            p(if (isReal) 0.0 else 60.0)
            d(4.0)

//            Feedback.SensorToMechanismRatio = 1.0 / (10.0 / 233.0)
//            motionMagic(2.1, 12.2)

            ClosedLoopGeneral.ContinuousWrap = true
        }
    }



    override fun periodic() {
        if (isSim) {
            if (isShooting) {
                if (i > 1) {
                    i = 0
                    shootSimulatedFuel()
                } else {
                    i++
                }
            }
            fuel.forEach { it.update() }
            logFuel("fuel", *fuel.toTypedArray())
            fuel.removeFuel()
        }
    }


    fun shoot(): Command = runCommand {
        if (!FieldManager.inTrenchArea) {
            if (rampedUp) {
                isShooting = true
                Spindexer.currentState = Spindexer.State.ON
            } else {
                isShooting = false
                Spindexer.currentState = Spindexer.State.OFF
            }

            hoodAngleSetpoint = (
                if (AimUtils.aimingAtGoal)
                    hubAngleCurve.get(AimUtils.distanceToGoal)
                else
                    floorAngleCurve.get(AimUtils.distanceToGoal)
            ).degrees
        } else {
            isShooting = false
            Spindexer.currentState = Spindexer.State.OFF
            hoodAngleSetpoint = HOOD_STOW_SETPOINT.degrees
        }
    }.finallyRun {
        isShooting = false
        Spindexer.currentState = Spindexer.State.OFF
        hoodAngleSetpoint = HOOD_STOW_SETPOINT.degrees
    }


    fun rampUp(): Command = runCommand(Shooter) {
        shooterVelocitySetpoint = (if (AimUtils.aimingAtGoal) hubSpeedCurve.get(AimUtils.distanceToGoal) else floorSpeedCurve.get(AimUtils.distanceToGoal)).metersPerSecond
    }.finallyRun { rampDown() }

    fun rampDown(): Command = runOnceCommand(Shooter) {
        shooterVelocitySetpoint = 0.0.inchesPerSecond
    }


    fun shootSimulatedFuel() {
        val exitVelocity = shooterVelocity.asMetersPerSecond//hubSpeedCurve.get(AimUtils.distanceToGoal)
        val exitAngle = hoodAngle//hubAngleCurve.get(AimUtils.distanceToGoal).degrees
//        val angleToTarget = AimUtils.aimTarget.angleTo(Turret.turretPose)
        val velocity2d = Translation2d(exitVelocity * exitAngle.cos(), 0.0).rotateBy(Turret.fieldCentricAngle.asRotation2d)
        val turretVelocity = Translation2d(Turret.turretOffsetFromCenter.x * Drive.gyroYawRate.asRadiansPerSecond, Turret.turretOffsetFromCenter.y * Drive.gyroYawRate.asRadiansPerSecond).rotateBy(Drive.heading) + Drive.velocity
        fuel.add(FuelSim(
            Translation3d(Turret.turretTranslation.x, Turret.turretTranslation.y, 0.4),
            Translation3d(velocity2d.x + turretVelocity.x, velocity2d.y + turretVelocity.y, exitVelocity * exitAngle.sin())
        ))
    }

    val sysIDShooterRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            1.0.voltsPerSecond,
            7.0.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("Shooter_Left_Position", shooterMotor.position.valueAsDouble)
            Logger.recordOutput("Shooter_Left_Velocity", shooterMotor.velocity.valueAsDouble)
        },
        SysIdRoutine.Mechanism({ output: Voltage ->
            shooterMotor.setControl(VoltageOut(output.asVolts))
            /* also log the requested output for SysId */
            Logger.recordOutput("Shooter_Left_Voltage", output.asVolts + 0.0001 * Math.random())
        }, null, this)
    )
}