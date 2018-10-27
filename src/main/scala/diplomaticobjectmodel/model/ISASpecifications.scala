// See LICENSE for license details.

package freechips.rocketchip.diplomaticobjectmodel.model

sealed trait PrivilegedArchitectureExtension

case object MachineLevelISA extends PrivilegedArchitectureExtension
case object SupervisorLevelISA extends PrivilegedArchitectureExtension

object PrivilegedArchitectureExtensions {
  val baseSpecifications = Map[PrivilegedArchitectureExtension, String](
      MachineLevelISA -> "Machine-Level ISA",
      SupervisorLevelISA -> "Supervisor-Level ISA"
  )
}

object BaseExtensions {
  val baseSpecifications = Map[OMBaseInstructionSet, String](
    RV32E -> "RV32E Base Integer Instruction Set",
    RV32I -> "RV32I Base Integer Instruction Set",
    RV64I -> "RV64I Base Integer Instruction Set"
  )
}

object ISAExtensions {
  val isaExtensionSpecifications = Map[OMExtensionType, String](
    M -> "M Standard Extension for Integer Multiplication and Division",
    A -> "A Standard Extension for Atomic Instruction",
    F -> "F Standard Extension for Single-Precision Floating-Point",
    D -> "D Standard Extension for Double-Precision Floating-Point",
    C -> "C Standard Extension for Compressed Instruction",
    U -> "TODO This is not really correct. The RISC‑V Instruction Set Manual, Volume II: Privileged Architecture",
    S -> "Supervisor-Level ISA"
  )
}
