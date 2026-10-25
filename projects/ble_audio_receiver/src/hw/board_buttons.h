#ifndef BOARD_BUTTONS_H
#define BOARD_BUTTONS_H

/**
 * @brief Initialize Button 1 (sw0) GPIO interrupt and debounce worker.
 *        Pressing Button 1 terminates all streams, disconnects the active phone,
 *        purges all stored BLE bonds, and opens a fresh pairing window.
 */
int board_buttons_init(void);

#endif /* BOARD_BUTTONS_H */