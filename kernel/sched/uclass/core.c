// SPDX-License-Identifier: GPL-2.0
/*
 * UCLASS (Unified Capacity and Latency Aware Scheduling System)
 *
 * Shared feature/gating helpers used by wakeup and placement units.
 */

#include "../sched.h"

#include "uclass.h"

bool uclass_enabled(void)
{
	return sched_feat(UCLASS);
}
