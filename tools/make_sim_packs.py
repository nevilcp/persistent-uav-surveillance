from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional
import zipfile

ROOT = Path.cwd()
SIM_DIR = ROOT / "results"
FIG_DIR = ROOT / "figures"
OUT_DIR = ROOT / "sim_packs"
OUT_DIR.mkdir(exist_ok=True)


@dataclass
class RunArtifacts:
    tag_prefix: str
    base: Optional[Path]
    metrics: Optional[Path]
    soc_series: Optional[Path]
    recovery_metrics: Optional[Path]
    coverage_snaps: List[Path]
    final_coverage: Optional[Path]
    figures: Dict[str, Path]


def _latest(paths: List[Path]) -> Optional[Path]:
    return max(paths, key=lambda p: p.stat().st_mtime) if paths else None


def _find_run_by_prefix(prefix: str) -> Optional[RunArtifacts]:
    m = sorted(SIM_DIR.glob(f"{prefix}*_metrics.csv"))
    m = [p for p in m if not p.name.endswith("_recovery_metrics.csv")] or m
    if not m:
        return None
    metrics = m[-1]
    base = metrics.with_name(metrics.name.replace("_metrics.csv", ""))

    soc = sorted(SIM_DIR.glob(f"{base.name}_soc_timeseries.csv"))
    rec = sorted(SIM_DIR.glob(f"{base.name}_recovery_metrics.csv"))
    covs = sorted(SIM_DIR.glob(f"{base.name}_coverage_gaps_*.csv"))
    finals = sorted(SIM_DIR.glob(f"{base.name}_final_coverage_gaps_*.csv"))

    fig_types = [
        "coverage_time",
        "violations_time",
        "fleet_state",
        "cost_index",
        "revisit_percentiles",
        "revisit_hist_early",
        "revisit_hist_mid",
        "revisit_hist_late",
        "soc_traces",
    ]
    figs: Dict[str, Path] = {}
    for ft in fig_types:
        candidates = list(FIG_DIR.glob(f"{base.name}*_{ft}.png"))
        sel = _latest(candidates)
        if sel is not None:
            figs[ft] = sel

    return RunArtifacts(
        tag_prefix=prefix,
        base=base,
        metrics=metrics,
        soc_series=soc[-1] if soc else None,
        recovery_metrics=rec[-1] if rec else None,
        coverage_snaps=covs,
        final_coverage=finals[-1] if finals else None,
        figures=figs,
    )


def _zip_pack(name: str, files: List[Path], manifest_rows: List[str]) -> Path:
    zip_path = OUT_DIR / f"{name}.zip"
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as z:
        z.writestr("MANIFEST.txt", "\n".join(manifest_rows) + "\n")
        for f in files:
            if f and f.exists():
                z.write(f, arcname=f.relative_to(ROOT))
    return zip_path


def build_packs(prefixes: List[str]) -> Dict[str, Dict[str, str]]:
    index: Dict[str, Dict[str, str]] = {}
    for pref in prefixes:
        art = _find_run_by_prefix(pref)
        if art is None:
            print(f"No artifacts found for prefix '{pref}'")
            continue
        files: List[Path] = []
        manifest: List[str] = []

        def add(p: Optional[Path], label: str):
            if p is None:
                return
            files.append(p)
            manifest.append(
                f"{label}: {p.relative_to(ROOT)} ({p.stat().st_size} bytes)"
            )

        add(art.metrics, "metrics")
        add(art.soc_series, "soc_series")
        add(art.recovery_metrics, "recovery_metrics")
        for c in art.coverage_snaps:
            add(c, "coverage_snap")
        add(art.final_coverage, "final_coverage")
        for k, v in art.figures.items():
            add(v, f"figure:{k}")

        pack_name = (
            f"{art.base.name}_pack" if art.base else pref.replace("*", "_") + "_pack"
        )
        zip_path = _zip_pack(pack_name, files, manifest)
        index[pref] = {
            "zip": str(zip_path.relative_to(ROOT)),
            "base": str(art.base.relative_to(ROOT)) if art.base else "",
            "metrics": str(art.metrics.relative_to(ROOT)) if art.metrics else "",
        }
        print(f"Wrote {zip_path}")

    with open(OUT_DIR / "index.json", "w", encoding="utf-8") as f:
        json.dump(index, f, indent=2)
    return index


if __name__ == "__main__":
    default_prefixes = [
        "sim_027_roundrobin_",
        "sim_028_roundrobin_",
        "sim_029_roundrobin_",
        "sim_030_roundrobin_",
        "sim_031_roundrobin_",
    ]
    build_packs(default_prefixes)
