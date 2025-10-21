package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.util.TeleOpBase

@TeleOp(name = "Hang Test", group = "Tests")
class HangTest : TeleOpBase(DriveSubsystem, IntakeSubsystem, OuttakeSubsystem) {
    private var hang2 = false
    private var hang3 = false
    private val hangCommand
        get() = if (!hang2) {
            if (OuttakeSubsystem.slidesState == OuttakeSubsystem.SlidesState.HANG_PREPARE)
                SequentialGroup(
                    OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN),
                    Delay(1.0),
                    OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HANG_PREPARE),
                    InstantCommand { hang2 = true }
                )
            else SequentialGroup(
                OuttakeSubsystem.setGearboxState(OuttakeSubsystem.GearboxState.HANG),
                Delay(0.6),
                OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HANG_PREPARE),

                IntakeSubsystem.setLiftPosition(0.1),
                IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.VERTICAL),
                IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),
                IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER)
            )
        } else {
            if (!hang3) {
                SequentialGroup(
                    InstantCommand { hang3 = true }
                )
            } else {
                InstantCommand {
                    hang2 = false
                    hang3 = false
                }
            }
        }

    override fun initialize() {
        driver1.cross.pressedCommand = { hangCommand }
    }

    override fun onStartButtonPressed() {
        IntakeSubsystem.initializeSystem()
        OuttakeSubsystem.initializeSystem()
    }
}