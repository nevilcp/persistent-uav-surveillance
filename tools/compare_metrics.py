import csv, json, math, os, statistics as st


def read_metrics(path: str):
    data = {}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            for k, v in row.items():
                k = k.strip().lower()
                data.setdefault(k, [])
                try:
                    data[k].append(float(v))
                except Exception:
                    data[k].append(float("nan"))
    return data


def summarize(m: dict, t_fail: float | None = None):
    t = m["time"]
    cov = m["coverage_%"]
    roll = m.get("rolling_avg_%", [float("nan")] * len(t))
    overdue = m.get("cells_overdue", [float("nan")] * len(t))
    active = m.get("active_uavs", [0.0] * len(t))
    swapping = m.get("swapping_uavs", [0.0] * len(t))
    deployed = [a + s for a, s in zip(active, swapping)]

    dt = [t[0]] + [max(1.0, t[i] - t[i - 1]) for i in range(1, len(t))]

    def fmean(xs):
        xs2 = [x for x in xs if not math.isnan(x)]
        return st.fmean(xs2) if xs2 else float("nan")

    out = {
        "duration_s": t[-1] - t[0],
        "avg_cov": fmean(cov),
        "peak_cov": max(cov),
        "min_cov": min(cov),
        "time_ge_90_s": sum(dt[i] for i, cv in enumerate(cov) if cv >= 90.0),
        "avg_overdue": fmean(overdue),
        "max_overdue": (
            max([x for x in overdue if not math.isnan(x)])
            if not all(math.isnan(x) for x in overdue)
            else float("nan")
        ),
        "avg_deployed": fmean(deployed),
        "max_deployed": max(deployed),
    }

    if t_fail is not None:
        idx = 0
        while idx < len(t) and t[idx] < t_fail:
            idx += 1
        post_cov = cov[idx:]
        post_over = overdue[idx:]
        out.update(
            {
                "post_min_cov": (min(post_cov) if post_cov else float("nan")),
                "post_max_overdue": (
                    max([x for x in post_over if not math.isnan(x)])
                    if post_over
                    else float("nan")
                ),
                "post_time_under_90_s": sum(
                    dt[i] for i in range(idx, len(t)) if cov[i] < 90.0
                ),
            }
        )
        rec = float("nan")
        for j in range(idx, len(t)):
            rv = roll[j]
            if not math.isnan(rv) and rv >= 90.0:
                rec = t[j] - t_fail
                break
        out["rec_time_to_90_roll_s"] = rec
    return out


base20 = "results/sim_027_roundrobin_20260914_190727_metrics.csv"
base23 = "results/sim_031_roundrobin_FAIL-time-u03-t1800_20260914_191413_metrics.csv"
rec23 = "results/sim_031_roundrobin_FAIL-time-u03-t1800_20260914_191413_recovery_metrics.csv"

m20 = read_metrics(base20)
m23 = read_metrics(base23)
s20 = summarize(m20)
s23 = summarize(m23, t_fail=1800.0)
rec_extra = None
if os.path.exists(rec23):
    with open(rec23, newline="") as f:
        r = list(csv.DictReader(f))
        if r:
            row = r[0]

            def f2(x):
                try:
                    return float(x)
                except Exception:
                    return float("nan")

            rec_extra = {
                "t_fail": f2(row.get("t_fail", "")),
                "min_cov_post_fail": f2(row.get("min_coverage_post_fail", "")),
                "time_under_90_s": f2(row.get("time_under_90_s", "")),
                "recovery_time_to_90_s": f2(row.get("recovery_time_to_90_s", "")),
            }

print(json.dumps({"sim20": s20, "sim23": s23, "sim23_recovery": rec_extra}, indent=2))
