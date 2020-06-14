#ifndef __FACE_CONTROL_H
#define __FACE_CONTROL_H

#include <Stdbool.h>

extern bool Mgk_PWM_Complete;

extern void Mgk_Ctrl_Face_Output(uint16_t face);
extern void Mgk_Ctrl_Face_Dir(bool dir);
extern void delay_us(uint16_t i);   
extern void Mgk_Ctrl_Fulse_Output(bool out);
#endif

