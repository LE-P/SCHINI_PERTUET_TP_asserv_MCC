#include "main.h"

#define encodeurDefinition 1024
#define Mid_Period_TIM2 2.147483647E9

#define GAIN_COURANT_HACHEUR 12 //normalement 12 mais par l'experience, le gain du hacheur est faux, ce qui limitera le courant admissible en entrée à 2.7 A

void Swing();
void Init_Onduleur();
void SetAlpha(char * argv);
void setPWM(int alpha);
void SetCurrent(char * argv);
void SetSpeed(char * argv);
float GetCurrent();
float GetSpeed(int timercoef);
void ReadEncodeur();
void ReadSpeed();
void PrintData();
