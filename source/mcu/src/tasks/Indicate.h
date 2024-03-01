#pragma once

#include <stdbool.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <common/config.h>
#include <drivers/skifio.h>
#include <tasks/stats.h>
#include <device/MPS.h>

#include "config.h"

void indication_run(PS_Control *MPS);
