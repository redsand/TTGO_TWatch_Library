#ifndef SETTINGS_H
#define SETTINGS_H

#include <LilyGoLib.h>
#include <vector>
#include "radar.h"

void settings_open(LilyGoLib** watch, int* threshold, std::vector<SignalSource>* history);

#endif // SETTINGS_H