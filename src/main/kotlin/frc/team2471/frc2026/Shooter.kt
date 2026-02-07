package frc.team2471.frc2026

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team2471.frc2026.AimUtils.generateShooterCurve
import frc.team2471.frc2026.Turret.turretMotor
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
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.units.asInches
import org.team2471.frc.lib.units.asInchesPerSecond
import org.team2471.frc.lib.units.asRadiansPerSecond
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.units.cos
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.inchesPerSecond
import org.team2471.frc.lib.units.metersPerSecond
import org.team2471.frc.lib.units.seconds
import org.team2471.frc.lib.units.sin
import org.team2471.frc.lib.units.volts
import org.team2471.frc.lib.units.voltsPerSecond
import org.team2471.frc.lib.util.angleTo
import kotlin.math.absoluteValue

// Unless otherwise specified every double here is in meters
object Shooter: SubsystemBase("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    val shooterMotor = TalonFX(Falcons.SHOOTER_0)
    val hoodMotor = TalonFX(Falcons.SHOOTER_HOOD)

    val WHEEL_DIAMETER = 4.0.inches

    @get:AutoLogOutput(key = "Shooter/Shooter Setpoint")
    var shooterVelocitySetpoint: LinearVelocity = 0.0.inchesPerSecond
        set(value) {
            field = value
            shooterMotor.setControl(VelocityVoltage(field.asInchesPerSecond/(WHEEL_DIAMETER.asInches * Math.PI)))
        }

    var hoodAngleSetpoint: Angle = 0.0.degrees
        set(value) {
            field = value.coerceIn(0.0.degrees, 45.0.degrees)
            hoodMotor.setControl(PositionVoltage(field))
        }

    @get:AutoLogOutput(key = "Shooter/Shooter Velocity")
    val shooterVelocity: LinearVelocity
        get() = (shooterMotor.velocity.valueAsDouble * WHEEL_DIAMETER.asInches * Math.PI).inchesPerSecond

    @get:AutoLogOutput(key = "Shooter/Shooter Current")
    val shooterCurrent: Double
        get() = shooterMotor.supplyCurrent.valueAsDouble

    var fuel: MutableList<FuelSim> = mutableListOf()

    val hubSpeedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(0.0, 5.639590955882352)
        put(0.30479999999999996, 5.650979709055888)
        put(0.6095999999999999, 5.685009079995864)
        put(0.9144, 5.741276500229907)
        put(1.2191999999999998, 5.819136921359697)
        put(1.524, 6.066392463116945)
        put(1.8288, 6.181519310750068)
        put(2.1336, 6.338269232046272)
        put(2.4383999999999997, 6.512421952188193)
        put(2.7432, 6.702687682604051)
        put(3.048, 6.907788978272454)
        put(3.3528, 7.089296123514534)
        put(3.6576, 7.31292483897281)
        put(3.9623999999999997, 7.54765278062865)
        put(4.2672, 7.792495041829245)
        put(4.572, 8.046543067831271)
        put(4.876799999999999, 8.476694499121312)
        put(5.1815999999999995, 8.75664077650401)
        put(5.4864, 9.043411750611908)
        put(5.7912, 8.952452495096281)
        put(6.096, 8.93445084533094)
        put(6.400799999999999, 9.899771725705317)
        put(6.7056, 10.145023838731676)
        put(7.0104, 10.477618443506287)
        put(7.3152, 10.881901567444025)
        put(7.62, 11.237952381734907)
        put(7.924799999999999, 11.466093689584005)
        put(8.2296, 11.875444490686492)
        put(8.5344, 12.190364171380546)
        put(8.839199999999998, 12.532097625267989)
        put(9.144, 12.774213842028066)
        put(9.448799999999999, 13.222989421986771)
        put(9.753599999999999, 13.475338604833617)
        put(10.058399999999999, 13.955125017762596)
        put(10.363199999999999, 14.209593673270334)
        put(10.668, 14.911570096432257)
        put(10.9728, 14.893900445371388)
        put(11.277599999999998, 15.265533542689216)
        put(11.5824, 15.558429401370308)
        put(11.887199999999998, 15.917418790321467)
        put(12.192, 16.343740018770976)
    }
    val hubAngleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(0.0, 90.0)
        put(0.30479999999999996, 86.36179796977044)
        put(0.6095999999999999, 82.7527005706532)
        put(0.9144, 79.20043921987744)
        put(1.2191999999999998, 75.7301569570825)
        put(1.524, 71.63919252263383)
        put(1.8288, 68.51573680801226)
        put(2.1336, 65.43403275355804)
        put(2.4383999999999997, 62.50911971695068)
        put(2.7432, 59.74323004172886)
        put(3.048, 57.135549524610795)
        put(3.3528, 54.7341105592441)
        put(3.6576, 52.42527035456388)
        put(3.9623999999999997, 50.2567718238655)
        put(4.2672, 48.221811338798496)
        put(4.572, 46.31307536090758)
        put(4.876799999999999, 44.53246172268135)
        put(5.1815999999999995, 42.887859957661114)
        put(5.4864, 41.34635908574173)
        put(5.7912, 39.68131513766788)
        put(6.096, 38.03505131362643)
        put(6.400799999999999, 37.23845117054059)
        put(6.7056, 35.97998952594711)
        put(7.0104, 34.86297550175815)
        put(7.3152, 33.88580197315637)
        put(7.62, 32.92331776583621)
        put(7.924799999999999, 31.86938774288247)
        put(8.2296, 31.07257799386358)
        put(8.5344, 30.210135645082943)
        put(8.839199999999998, 29.422201592597148)
        put(9.144, 28.542379859138137)
        put(9.448799999999999, 27.9703448394974)
        put(9.753599999999999, 27.16969882117106)
        put(10.058399999999999, 26.70912983401122)
        put(10.363199999999999, 25.969882543552025)
        put(10.668, 25.86958023156304)
        put(10.9728, 24.810098852722597)
        put(11.277599999999998, 24.308219755648352)
        put(11.5824, 23.716303902060595)
        put(11.887199999999998, 23.239397070794915)
        put(12.192, 22.87842254755574)
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

    var isShooting = false
    var i = 0

    init {
        shooterMotor.applyConfiguration {
            currentLimits(25.0, 30.0, 1.0)
            coastMode()

            p(0.0)
            s(0.0, StaticFeedforwardSignValue.UseVelocitySign)
        }
        shooterMotor.addFollower(Falcons.SHOOTER_1)

        hoodMotor.applyConfiguration {
            currentLimits(25.0, 30.0, 1.0)
            inverted(true)
            brakeMode()
            s(0.13, StaticFeedforwardSignValue.UseClosedLoopSign)
            p(0.0)

//            Feedback.SensorToMechanismRatio = 1.0 / (10.0 / 233.0)
//            motionMagic(2.1, 12.2)

            ClosedLoopGeneral.ContinuousWrap = true
        }
    }



    override fun periodic() {
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


    fun shoot(): Command = runCommand(Shooter) {
        shooterVelocitySetpoint = (if (AimUtils.aimingAtGoal) hubSpeedCurve.get(AimUtils.distanceToGoal) else floorSpeedCurve.get(AimUtils.distanceToGoal)).metersPerSecond

        hoodAngleSetpoint = (if (AimUtils.aimingAtGoal) hubAngleCurve.get(AimUtils.distanceToGoal) else floorAngleCurve.get(AimUtils.distanceToGoal)).degrees

    }.finallyRun(rampDown())


    fun rampDown(): Command = runOnceCommand(Shooter) {
        shooterMotor.setControl(VelocityVoltage(0.0))
    }


    fun shootSimulatedFuel() {
        val robotVelocity = Drive.velocity

        val exitVelocity = hubSpeedCurve.get(AimUtils.distanceToGoal)
        val exitAngle = hubAngleCurve.get(AimUtils.distanceToGoal).degrees
        val angleToTarget = AimUtils.aimTarget.angleTo(Turret.turretPose)
        val velocity2d = Translation2d(-exitVelocity * exitAngle.cos(), 0.0).rotateBy(angleToTarget.asRotation2d)
        val turretVelocity = Translation2d(Turret.turretOffsetFromCenter.x * Drive.gyroYawRate.asRadiansPerSecond, Turret.turretOffsetFromCenter.y * Drive.gyroYawRate.asRadiansPerSecond).rotateBy(Drive.heading) + Drive.velocity
        fuel.add(FuelSim(
            Translation3d(Turret.turretPose.x, Turret.turretPose.y, 0.4),
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