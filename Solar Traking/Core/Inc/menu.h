// File: menu.h
#ifndef MENU_H_
#define MENU_H_

#include "main.h" // Giả sử đây là tệp header chứa tất cả các khai báo HAL

typedef enum {
    MENU_MANUAL,
    MENU_AUTO,
    MENU_SETUP,
    MENU_NUM_STATES
} MenuState;

void processMenuState(MenuState *currentMenu);
void manualMenuHandler(TIM_HandleTypeDef *htim);
void autoMenuHandler(void);
void setupMenuHandler(void);

#endif // MENU_H_
