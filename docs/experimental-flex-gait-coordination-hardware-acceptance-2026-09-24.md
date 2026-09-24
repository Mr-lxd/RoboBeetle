# Experimental Flex gait and coordination hardware acceptance — 2026-09-24

## Scope

Accepted image HEAD: `bf0c73c5fdfb11236a85e04e0138407c58169b29` on `codex/motion-gait-flex-coordination`. This record covers the listed software, deployment, real-link selector, and powered air-bench checks only.

## Software and deployment

- Firmware host gate: 37 executable tests PASS; 13 compile-contract objects PASS; USART2 source/config contract PASS.
- Raspberry Pi Linux: native aarch64 Release build PASS; full CTest 10/10 PASS; focused gateway/application tests 5/5 PASS. `robobeetle_pi_gateway` was generated as an ARM aarch64 ELF, the installed service was active, TCP `0.0.0.0:47000` was listening, and the installed binary SHA256 matched the tested build binary.
- STM32 Debug ARM CMake build PASS. CMake memory report: RAM 6216 B / 128 KB = 4.74%; FLASH 67316 B / 512 KB = 12.84%. `arm-none-eabi-size`: text 67224, data 92, bss 6120, dec 73436.
- CMSIS-DAPv2 / SWD detected an STM32F4 Cortex-M4 at 100 kHz. Programming reported `Programming Finished`; verification reported `Verify Started` and `Verified OK`; reset/run passed. A subsequent halt observed PC `0x080059d4`, and execution resumed successfully.
- Qt Release build PASS; `windeployqt` PASS; reduced-PATH 5-second portable launch PASS: `D:\RoboBeetleConsole-portable-motion-gait-flex\RoboBeetleConsole.exe`.

## Real-link selector acceptance

With Servo Power OFF, Experimental Flex, Same Direction, and Opposite Direction each displayed Confirmed. Selector activity did not start Motion automatically. CRC = 0; Timeout = 0.

## Powered air-bench acceptance

| Check | Result |
| --- | --- |
| SimpleGait + Same Direction | Front/rear physical movement in the same direction: PASS |
| SimpleGait + Opposite Direction | Front/rear physical movement in opposite directions: PASS |
| STOP | Smooth return to neutral: PASS |
| Experimental Flex | Nominal cycle about 2 seconds; visually clear 1300 ms slow stroke / 700 ms fast recovery; continuous motion; smooth STOP to neutral: PASS |
| Experimental Flex + Opposite Direction | Rear physical relationship reversed: PASS |
| Turn Left | FrontLeft + RearLeft actively hold logical neutral while the right side continues gait: PASS |
| Turn Right | FrontRight + RearRight actively hold logical neutral while the left side continues gait: PASS |
| While Motion is active | Gait and Front/Rear selectors disabled: PASS |

Final CRC = 0; Timeout = 0.

## Water-validation boundary

The following remain Pending Water Validation: water propulsion, hydrodynamic efficiency, final power-stroke direction, flexible-paddle deformation effectiveness, and Turn effectiveness in water. The air-bench and real-link results above do not establish these outcomes.
