@file:Suppress("unused")

package frc.team2471.frc2026

import org.team2471.frc.lib.util.isSim

object Sparks {
}

object AnalogSensors {
}

object DigitalSensors {
}

object Falcons {
    const val FRONT_RIGHT_DRIVE = 23
    const val FRONT_RIGHT_STEER = 14

    const val FRONT_LEFT_DRIVE = 16
    const val FRONT_LEFT_STEER = 10

    const val BACK_RIGHT_DRIVE = 22
    val BACK_RIGHT_STEER = if (isSim) 40 else 15

    val BACK_LEFT_DRIVE = if (isSim) 41 else 18
    const val BACK_LEFT_STEER = 11



    const val INTAKE_DEPLOY = 59
    const val INTAKE_ROLLER_0 = 58
    const val INTAKE_ROLLER_1 = 60

    const val SHOOTER_HOOD = 53
    const val SHOOTER_0 = 57
    const val SHOOTER_1 = 56

    const val TURRET_0 = 55
    const val TURRET_1 = 54

    const val SPIN_0 = 52
    const val SPIN_1 = 51
    const val SIDETAKE = 49
    const val UPTAKE = 48
}

object Talons {
}

object CANCoders {
    const val FRONT_RIGHT = 25
    const val FRONT_LEFT = 27
    const val BACK_RIGHT = 26
    const val BACK_LEFT = 28
}

object PWMPort {
}

object ServoPort {
}

object CANivores {
}

object CANSensors {
    const val PIGEON = 2
}

object I2CPort {
}