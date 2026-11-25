# RUMBLE

Remote-controlled car on an STM32L152RE (Nucleo), driven from a PC
gamepad over Bluetooth.

## Layout

| Path | What |
|---|---|
| `Core/`, `Drivers/`, `RUMBLE.ioc`, `*.ld`, `.project`, `.cproject` | STM32CubeIDE project (firmware) |
| `Core/Src/main.c` | firmware logic: Bluetooth packets on USART1, debug on USART2, motor PWM on TIM3, wheel encoder on TIM2, reverse buzzer on TIM10 |
| `pc_controller/` | Python gamepad controller that runs on the PC — **not** opened in CubeIDE, see its README |

## Opening the firmware in STM32CubeIDE

First time: File → Import → General → Existing Projects into Workspace →
select this folder → Finish (leave "Copy projects into workspace" off).

Already imported RUMBLE before? After switching branches just right-click
the project → Refresh (F5).

Build output (`Debug/`, `Release/`) and per-user IDE files are not
committed.
