// SPDX-License-Identifier: GPL-2.0
/*
 * smooth_step.c - Adaptive thermal throttling governor
 *
 * This governor keeps the device-tree and cooling-device model used by
 * step_wise, but derives the requested cooling state from the amount of
 * thermal overshoot instead of blindly moving one state per sample.
 */

#include <linux/thermal.h>
#include <trace/events/thermal.h>

#include "thermal_core.h"

#define SMOOTH_STEP_DEFAULT_WINDOW_MC 5000

static int get_trip_temperature(struct thermal_zone_device *tz, int trip)
{
	int trip_temp;

	if (trip == THERMAL_TRIPS_NONE)
		return tz->forced_passive;

	if (tz->ops->get_trip_temp(tz, trip, &trip_temp))
		return tz->temperature;

	return trip_temp;
}

static int get_hysteresis_temperature(struct thermal_zone_device *tz, int trip,
				      int trip_temp)
{
	int hyst;

	if (trip == THERMAL_TRIPS_NONE || !tz->ops->get_trip_hyst)
		return trip_temp;

	if (tz->ops->get_trip_hyst(tz, trip, &hyst))
		return trip_temp;

	return trip_temp - hyst;
}

static int get_control_window(struct thermal_zone_device *tz, int trip,
			      int trip_temp, int hyst_temp)
{
	int i, next_temp, window = trip_temp - hyst_temp;

	for (i = 0; i < tz->trips; i++) {
		if (i == trip || tz->ops->get_trip_temp(tz, i, &next_temp))
			continue;

		if (next_temp > trip_temp) {
			int delta = next_temp - trip_temp;

			if (!window || delta < window)
				window = delta;
		}
	}

	if (window <= 0)
		window = SMOOTH_STEP_DEFAULT_WINDOW_MC;

	return window;
}

static unsigned long
clamp_target_state(struct thermal_instance *instance, unsigned long target)
{
	if (target < instance->lower)
		return instance->lower;

	if (target > instance->upper)
		return instance->upper;

	return target;
}

static unsigned long get_target_state(struct thermal_zone_device *tz,
				      struct thermal_instance *instance,
				      enum thermal_trend trend, bool throttle,
				      int trip_temp, int hyst_temp)
{
	struct thermal_cooling_device *cdev = instance->cdev;
	unsigned long cur_state, span, target;
	int window, overshoot;

	if (!throttle && instance->target == THERMAL_NO_TARGET)
		return THERMAL_NO_TARGET;

	cdev->ops->get_cur_state(cdev, &cur_state);
	target = instance->target;
	dev_dbg(&cdev->device, "cur_state=%ld\n", cur_state);

	if (throttle) {
		if (trend == THERMAL_TREND_RAISE_FULL)
			return instance->upper;

		span = instance->upper - instance->lower;
		window = get_control_window(tz, instance->trip, trip_temp, hyst_temp);
		overshoot = max(tz->temperature - trip_temp, 0);

		target = instance->lower;
		if (span)
			target += DIV_ROUND_UP(span * overshoot, window);

		target = clamp_target_state(instance, target);

		if (trend == THERMAL_TREND_RAISING && target <= cur_state &&
		    cur_state < instance->upper)
			target = cur_state + 1;
		else if (trend == THERMAL_TREND_STABLE && target < cur_state)
			target = cur_state;

		return clamp_target_state(instance, target);
	}

	if (trend == THERMAL_TREND_DROP_FULL || cur_state <= instance->lower ||
	    instance->target <= instance->lower)
		return THERMAL_NO_TARGET;

	return clamp_target_state(instance, cur_state - 1);
}

static void update_passive_instance(struct thermal_zone_device *tz,
				    enum thermal_trip_type type, int value)
{
	if (type == THERMAL_TRIP_PASSIVE || type == THERMAL_TRIPS_NONE)
		tz->passive += value;
}

static void thermal_zone_trip_update(struct thermal_zone_device *tz, int trip)
{
	int trip_temp, hyst_temp, old_target;
	unsigned long target;
	enum thermal_trip_type trip_type;
	enum thermal_trend trend;
	struct thermal_instance *instance;
	bool throttle;

	trip_temp = get_trip_temperature(tz, trip);
	hyst_temp = get_hysteresis_temperature(tz, trip, trip_temp);
	trip_type = THERMAL_TRIPS_NONE;
	if (trip != THERMAL_TRIPS_NONE)
		tz->ops->get_trip_type(tz, trip, &trip_type);

	trend = get_tz_trend(tz, trip);

	mutex_lock(&tz->lock);

	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		if (instance->trip != trip)
			continue;

		old_target = instance->target;
		throttle = tz->temperature >= trip_temp ||
			(tz->temperature > hyst_temp &&
			 old_target != THERMAL_NO_TARGET);

		target = get_target_state(tz, instance, trend, throttle,
					  trip_temp, hyst_temp);
		instance->target = target;
		dev_dbg(&instance->cdev->device, "old_target=%d, target=%d\n",
			old_target, (int)instance->target);

		if (instance->initialized && old_target == instance->target)
			continue;

		if (!instance->initialized) {
			if (instance->target != THERMAL_NO_TARGET) {
				trace_thermal_zone_trip(tz, trip, trip_type, true);
				update_passive_instance(tz, trip_type, 1);
			}
		} else if (old_target == THERMAL_NO_TARGET &&
			   instance->target != THERMAL_NO_TARGET) {
			trace_thermal_zone_trip(tz, trip, trip_type, true);
			update_passive_instance(tz, trip_type, 1);
		} else if (old_target != THERMAL_NO_TARGET &&
			   instance->target == THERMAL_NO_TARGET) {
			trace_thermal_zone_trip(tz, trip, trip_type, false);
			update_passive_instance(tz, trip_type, -1);
		}

		instance->initialized = true;
		mutex_lock(&instance->cdev->lock);
		instance->cdev->updated = false;
		mutex_unlock(&instance->cdev->lock);
	}

	mutex_unlock(&tz->lock);
}

static int smooth_step_throttle(struct thermal_zone_device *tz, int trip)
{
	struct thermal_instance *instance;

	thermal_zone_trip_update(tz, trip);

	if (tz->forced_passive)
		thermal_zone_trip_update(tz, THERMAL_TRIPS_NONE);

	mutex_lock(&tz->lock);

	list_for_each_entry(instance, &tz->thermal_instances, tz_node)
		thermal_cdev_update(instance->cdev);

	mutex_unlock(&tz->lock);

	return 0;
}

static struct thermal_governor thermal_gov_smooth_step = {
	.name		= "smooth_step",
	.throttle	= smooth_step_throttle,
};

int thermal_gov_smooth_step_register(void)
{
	return thermal_register_governor(&thermal_gov_smooth_step);
}

void thermal_gov_smooth_step_unregister(void)
{
	thermal_unregister_governor(&thermal_gov_smooth_step);
}
