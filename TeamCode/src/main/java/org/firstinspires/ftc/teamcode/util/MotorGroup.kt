@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

object MotorGroup {
    fun List<DcMotorEx>.resetEncodersN(backTo: DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        this.forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = backTo
        }
    }

    fun List<DcMotorEx>.setModeN(mode: DcMotor.RunMode) {
        this.forEach { motor -> motor.mode = mode }
    }

    fun List<DcMotorEx>.setBrakeModeN(mode: DcMotor.ZeroPowerBehavior) {
        this.forEach { motor -> motor.zeroPowerBehavior = mode }
    }

    fun List<DcMotorEx>.setPowerN(power: Double) {
        this.forEach { motor -> motor.power = power }
    }

    fun List<DcMotorEx>.getPowersN(): List<Double> {
        return this.map { motor -> motor.power }
    }

    fun List<DcMotorEx>.getCurrentsN(): List<Double> {
        return this.map { motor -> motor.getCurrent(CurrentUnit.AMPS) }
    }

    fun List<MotorEx>.resetEncoders(backTo: DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
        this.forEach { motor ->
            motor.resetEncoder()
        }
    }

    fun List<MotorEx>.setMode(mode: DcMotor.RunMode) {
        this.forEach { motor -> motor.motor.mode = mode }
    }

    fun List<MotorEx>.setBrakeMode(mode: DcMotor.ZeroPowerBehavior) {
        this.forEach { motor -> motor.motor.zeroPowerBehavior = mode }
    }

    fun List<MotorEx>.setPower(power: Double) {
        this.forEach { motor -> motor.power = power }
    }

    fun List<MotorEx>.getPowers(): List<Double> {
        return this.map { motor -> motor.power }
    }

    fun List<MotorEx>.getCurrents(): List<Double> {
        return this.map { motor -> motor.motor.getCurrent(CurrentUnit.AMPS) }
    }
}
