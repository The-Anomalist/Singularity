#!/system/bin/sh
# SPDX-License-Identifier: GPL-2.0
# Observational scheduler/runtime diagnostics for Geekbench on Kona.

set -u

INTERVAL_MS=200
SAMPLES=150
OUT="/data/local/tmp/kona-geekbench-diag-$(date +%Y%m%d-%H%M%S)"
PIN_CPU=""
TRACE=0
TARGET="com.primatelabs.geekbench6"

usage()
{
	cat <<EOF
Usage: $0 [--interval-ms N] [--samples N] [--out DIR] [--trace]
          [--pin-hot 6|7]

The default mode only reads kernel interfaces.  --pin-hot is an explicit,
temporary A/B-test option; it applies taskset to each interval's hottest TID.
The original affinity is recorded but cannot safely be restored if Geekbench
replaces that worker, so use this option for a dedicated benchmark run only.
EOF
}

while [ "$#" -gt 0 ]; do
	case "$1" in
	--interval-ms) INTERVAL_MS=$2; shift 2 ;;
	--samples) SAMPLES=$2; shift 2 ;;
	--out) OUT=$2; shift 2 ;;
	--trace) TRACE=1; shift ;;
	--pin-hot) PIN_CPU=$2; shift 2 ;;
	-h|--help) usage; exit 0 ;;
	*) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
	esac
done

case "$INTERVAL_MS:$SAMPLES" in
	*[!0-9:]*) echo "interval and samples must be integers" >&2; exit 2 ;;
esac
case "$PIN_CPU" in
	""|6|7) ;;
	*) echo "--pin-hot accepts only 6 or 7" >&2; exit 2 ;;
esac

mkdir -p "$OUT" || exit 1
TMP="$OUT/.tmp"
mkdir -p "$TMP"
trap 'stop_trace; rm -rf "$TMP"' EXIT HUP INT TERM

log()
{
	printf '%s %s\n' "$(date +%s.%N 2>/dev/null || date +%s)" "$*" |
		tee -a "$OUT/run.log"
}

read_file()
{
	[ -r "$1" ] && { echo "### $1"; cat "$1"; }
}

snapshot_irq()
{
	cat /proc/interrupts > "$1.interrupts"
	cat /proc/softirqs > "$1.softirqs"
}

# Emit numeric CPU-column deltas while preserving the interrupt description.
delta_table()
{
	awk '
	NR==FNR {
		if (NR == 1) ncpu=NF;
		key=$1; sub(/:$/, "", key);
		for (i=2; i<=ncpu+1; i++) old[key,i]=$i+0;
		next
	}
	FNR==1 { ncpu=NF; printf "source"; for(i=0;i<ncpu;i++) printf " CPU%d",i; print " description"; next }
	{
		key=$1; sub(/:$/, "", key);
		printf "%s", key;
		for (i=2; i<=ncpu+1; i++) printf " %d", ($i+0)-old[key,i];
		printf " "; for (i=ncpu+2; i<=NF; i++) printf "%s%s", $i, (i<NF?OFS:ORS)
	}' "$1" "$2"
}

find_pid()
{
	pidof "$TARGET" 2>/dev/null | awk '{print $1}'
}

# /proc/TID/stat fields after the final ')' begin with field 3.  Therefore
# utime, stime, processor are remainder fields 12, 13 and 37 respectively.
task_stat()
{
	tid=$1
	line=$(cat "/proc/$PID/task/$tid/stat" 2>/dev/null) || return 1
	rest=${line##*) }
	set -- $rest
	eval "utime=\${12}; stime=\${13}; psr=\${37}"
	printf '%s %s\n' "$((utime + stime))" "$psr"
}

sched_value()
{
	awk -v key="$2" '$1 == key { print $3; exit }' "$1" 2>/dev/null
}

dump_static()
{
	{
		echo "kernel=$(uname -a)"
		echo "cmdline=$(cat /proc/cmdline)"
		for cpu in /sys/devices/system/cpu/cpu[0-7]; do
			[ -d "$cpu" ] || continue
			echo "### ${cpu##*/}"
			for f in cpu_capacity online cpufreq/scaling_driver \
				cpufreq/scaling_governor cpufreq/scaling_cur_freq \
				cpufreq/scaling_min_freq cpufreq/scaling_max_freq \
				cpufreq/cpuinfo_max_freq cpufreq/dcvsh_freq_limit; do
				[ -r "$cpu/$f" ] && echo "$f=$(cat "$cpu/$f")"
			done
		done
		for d in /dev/cpuset/*; do
			[ -d "$d" ] || continue
			echo "cpuset=${d#/dev/cpuset/} cpus=$(cat "$d/cpus" 2>/dev/null)"
		done
		for d in /dev/stune/*; do
			[ -d "$d" ] || continue
			echo "stune=${d#/dev/stune/} boost=$(cat "$d/schedtune.boost" 2>/dev/null) prefer_idle=$(cat "$d/schedtune.prefer_idle" 2>/dev/null)"
		done
		for f in /sys/class/thermal/thermal_zone*/{type,temp,policy}; do read_file "$f"; done
		for f in /sys/devices/system/cpu/cpu[0-7]/cpuidle/state*/{name,latency,disable}; do read_file "$f"; done
	} > "$OUT/system.txt" 2>&1

	cat /proc/interrupts > "$OUT/interrupts.before"
	cat /proc/softirqs > "$OUT/softirqs.before"
	ps -A -T -o USER,PID,TID,PPID,CLS,RTPRIO,PRI,NI,PSR,STAT,COMM \
		> "$OUT/threads.before" 2>&1 || ps -A -T > "$OUT/threads.before" 2>&1
	for f in /proc/irq/*/smp_affinity /proc/irq/*/smp_affinity_list; do
		read_file "$f"
	done > "$OUT/irq_affinity.txt" 2>&1
	dmesg > "$OUT/dmesg.before" 2>&1
}

start_trace()
{
	[ "$TRACE" -eq 1 ] || return
	TRACEFS=/sys/kernel/tracing
	[ -d "$TRACEFS/events" ] || TRACEFS=/sys/kernel/debug/tracing
	[ -d "$TRACEFS/events" ] || { log "tracefs unavailable"; TRACE=0; return; }
	echo 0 > "$TRACEFS/tracing_on"
	echo > "$TRACEFS/trace"
	for e in sched/sched_switch sched/sched_wakeup sched/sched_waking \
		sched/sched_migrate_task power/cpu_frequency power/cpu_frequency_limits \
		dcvsh/dcvsh_freq; do
		[ -e "$TRACEFS/events/$e/enable" ] && echo 1 > "$TRACEFS/events/$e/enable"
	done
	echo 1 > "$TRACEFS/tracing_on"
	log "trace enabled at $TRACEFS"
}

stop_trace()
{
	[ "${TRACE:-0}" -eq 1 ] || return 0
	[ -n "${TRACEFS:-}" ] || return 0
	echo 0 > "$TRACEFS/tracing_on" 2>/dev/null || true
	cat "$TRACEFS/trace" > "$OUT/trace.txt" 2>/dev/null || true
	TRACE=0
}

dump_task()
{
	tid=$1
	base="/proc/$PID/task/$tid"
	{
		echo "pid=$PID tid=$tid"
		read_file "$base/status"
		read_file "$base/cgroup"
		read_file "$base/sched"
		read_file "$base/schedstat"
	} > "$OUT/hot-tid-$tid.txt" 2>&1
}

dump_static
snapshot_irq "$TMP/start"
start_trace
log "waiting for $TARGET; output=$OUT"

i=0
while [ "$i" -lt "$SAMPLES" ]; do
	PID=$(find_pid)
	if [ -z "$PID" ] || [ ! -d "/proc/$PID/task" ]; then
		sleep "0.$INTERVAL_MS" 2>/dev/null || sleep 1
		continue
	fi

	: > "$TMP/before"
	for t in /proc/$PID/task/[0-9]*; do
		tid=${t##*/}; stat=$(task_stat "$tid") || continue
		set -- $stat; echo "$tid $1 $2" >> "$TMP/before"
	done
	sleep "0.$INTERVAL_MS" 2>/dev/null || usleep "$((INTERVAL_MS * 1000))"

	hot_tid= hot_delta=-1 hot_psr=-1
	while read -r tid old_time old_psr; do
		stat=$(task_stat "$tid") || continue; set -- $stat
		delta=$(($1 - old_time))
		if [ "$delta" -gt "$hot_delta" ]; then
			hot_tid=$tid; hot_delta=$delta; hot_psr=$2
		fi
	done < "$TMP/before"
	[ -n "$hot_tid" ] || continue

	base="/proc/$PID/task/$hot_tid"
	allowed=$(awk '/^Cpus_allowed_list:/ {print $2}' "$base/status" 2>/dev/null)
	cpuset=$(awk -F: '$2 == "cpuset" || $2 == "" {print $3}' "$base/cgroup" 2>/dev/null | tail -1)
	policy=$(awk '/^policy/ {print $3; exit}' "$base/sched" 2>/dev/null)
	prio=$(awk '/^prio/ {print $3; exit}' "$base/sched" 2>/dev/null)
	nice=$(awk '/^nice/ {print $3; exit}' "$base/sched" 2>/dev/null)
	migrations=$(sched_value "$base/sched" nr_migrations)
	voluntary=$(sched_value "$base/sched" nr_voluntary_switches)
	involuntary=$(sched_value "$base/sched" nr_involuntary_switches)
	printf 'sample=%d pid=%s hot_tid=%s ticks=%s cpu=%s allowed=%s cpuset=%s policy=%s prio=%s nice=%s migrations=%s nvcsw=%s nivcsw=%s\n' \
		"$i" "$PID" "$hot_tid" "$hot_delta" "$hot_psr" "$allowed" "$cpuset" \
		"$policy" "$prio" "$nice" "$migrations" "$voluntary" "$involuntary" |
		tee -a "$OUT/hot-samples.txt"
	dump_task "$hot_tid"

	if [ -n "$PIN_CPU" ]; then
		mask=40; [ "$PIN_CPU" = 7 ] && mask=80
		taskset -p "$mask" "$hot_tid" >> "$OUT/pinning.txt" 2>&1
	fi
	i=$((i + 1))
done

stop_trace
snapshot_irq "$TMP/end"
delta_table "$TMP/start.interrupts" "$TMP/end.interrupts" > "$OUT/interrupts.delta"
delta_table "$TMP/start.softirqs" "$TMP/end.softirqs" > "$OUT/softirqs.delta"
dmesg > "$OUT/dmesg.after" 2>&1
dmesg | grep -iE 'dcvsh|thermal|thrott|cpufreq|frequency|limit' > "$OUT/thermal-dcvsh.log" 2>&1

awk '
{
	for(i=1;i<=NF;i++) { split($i,a,"="); v[a[1]]=a[2] }
	if (v["ticks"] > 0) ticks[v["cpu"]] += v["ticks"];
	if (last_tid == v["hot_tid"] && last_cpu != v["cpu"]) migrations++;
	last_tid=v["hot_tid"]; last_cpu=v["cpu"]; total+=v["ticks"]
}
END {
	printf "execution_residency_by_sampled_ticks:";
	for(cpu=0;cpu<8;cpu++) printf " cpu%d=%.1f%%",cpu,total?100*ticks[cpu]/total:0;
	printf "\nobserved_interval_cpu_changes=%d total_ticks=%d\n",migrations,total
}' "$OUT/hot-samples.txt" | tee "$OUT/summary.txt"

log "complete; inspect summary.txt, *.delta, hot-tid-*.txt and trace.txt"
