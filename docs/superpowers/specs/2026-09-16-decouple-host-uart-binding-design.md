# Host UART Binding Decoupling Design

## Goal

Make the single-instance STM32 host UART transport identify callbacks by the
`UART_HandleTypeDef *` passed to `uart_transport_stm32_init()` rather than by
the handle's peripheral `Instance`. The production target remains bound to
`&huart1`; this change is a prerequisite for later alternate-UART integration,
not an alternate-UART implementation.

## Scope and non-goals

- Keep the existing single transport instance, owned TX queue, interrupt-driven
  TX, RX re-arm, diagnostics, abort/recovery state machine, and all Protocol V2
  behavior unchanged.
- Change only the callback identity predicate and its host-test coverage.
- Do not modify `main.c`, `app_main_init(&huart1, ...)`, ISR dispatch, CubeMX
  configuration, `.ioc`, UART baud rates, queue sizing, motion, safety, or
  sensor code.
- Do not add USART2, `huart2`, PA2/PA3, Raspberry Pi code, or multi-instance
  transport support.

## Design

`uart_transport_stm32.c` will rename the USART-specific helper to
`uart_transport_matches_bound_uart()`. It returns true only when the callback
handle is non-NULL, the transport has a non-NULL bound handle, and both
pointers are identical. All RX-complete, TX-complete, abort-complete, and error
callbacks will use this predicate before touching transport state. No callback
path will inspect `huart->Instance` for identity.

The existing firmware initialization remains the source of the binding:
`app_main_init()` passes `&huart1`, and `main.c`/ISR/CubeMX files stay unchanged.
Therefore the current USART1 hardware path and its interrupt routing remain
identical.

## Verification design

The existing host HAL stub will accept any non-NULL UART handle for the
one-byte receive and interrupt transmit operations, allowing tests to model a
bound handle whose `Instance` is not USART1. The transport regression suite
will cover:

1. A bound non-USART1 handle successfully processes RX, TX completion, UART
   error, and abort/recovery callbacks.
2. A different handle with the same `Instance` is ignored and cannot alter
   RX/TX state, queue ownership, diagnostics, or recovery state.
3. NULL and other non-bound handles remain safely ignored.

Existing USART1 tests and the full firmware host suite remain required. An ARM
firmware build is required after the host regressions. No documentation change
is needed unless an existing statement explicitly claims that transport
identity is tied to the USART1 peripheral.
