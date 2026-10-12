// Metric catalogue + the sequential (single-hue, light→dark) colour ramp used by the Lab heatmap.
import type { ExperimentInfo, MetricSpec, Metrics, Summary } from './api';

export interface MetricDef {
	key: string;
	label: string;
	higherIsBetter: boolean;
	format: (v: number) => string;
	/** Fixed domain; null → derived from the data. */
	domain: [number, number] | null;
}

const pct = (v: number) => `${(v * 100).toFixed(1)}%`;

export const METRICS: MetricDef[] = [
	{ key: 'success', label: 'Reached behind', higherIsBetter: true, format: pct, domain: [0, 1] },
	{ key: 'time_s', label: 'Time to behind', higherIsBetter: false, format: (v) => `${v.toFixed(2)} s`, domain: null },
	{ key: 'wrong_touch', label: 'Touched ball first', higherIsBetter: false, format: pct, domain: [0, 1] },
	{ key: 'ball_push_wrong_cm', label: 'Ball pushed back', higherIsBetter: false, format: (v) => `${v.toFixed(1)} cm`, domain: null },
	{ key: 'path_eff', label: 'Path efficiency', higherIsBetter: true, format: (v) => v.toFixed(2), domain: [0, 1] },
	{ key: 'out_of_bounds', label: 'Went out of bounds', higherIsBetter: false, format: pct, domain: [0, 1] },
	{ key: 'wall_hit', label: 'Hit a wall', higherIsBetter: false, format: pct, domain: [0, 1] },
	{ key: 'score', label: 'Score', higherIsBetter: true, format: (v) => v.toFixed(1), domain: null }
];

export const metricDef = (key: string, defs: MetricDef[] = METRICS) =>
	defs.find((m) => m.key === key) ?? defs[0];

const UNIT_FORMAT: Record<string, (v: number) => string> = {
	pct,
	deg: (v) => `${v.toFixed(1)}°`,
	cm: (v) => `${v.toFixed(1)} cm`,
	'cm/s': (v) => `${v.toFixed(1)} cm/s`,
	s: (v) => `${v.toFixed(2)} s`,
	g: (v) => `${v.toFixed(3)} g`,
	Hz: (v) => `${v.toFixed(1)} Hz`
};

function fromSpec(s: MetricSpec): MetricDef {
	return {
		key: s.key,
		label: s.label,
		higherIsBetter: s.higher_is_better,
		format: UNIT_FORMAT[s.unit] ?? ((v: number) => (Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(2))),
		domain: s.domain ?? null
	};
}

/** The metrics an experiment declares (bucky.lab.experiments.base.Experiment.metric_specs),
 *  or the built-in drive metrics. */
export function metricsFor(exp: ExperimentInfo | undefined | null): MetricDef[] {
	return exp?.metrics?.length ? exp.metrics.map(fromSpec) : METRICS;
}

/** Columns for the summary tables. */
export function summaryColumns(exp: ExperimentInfo | undefined | null): MetricDef[] {
	if (exp?.metrics?.length) {
		const cols = exp.metrics.filter((m) => m.summary).map(fromSpec);
		return cols.length ? cols : exp.metrics.slice(0, 6).map(fromSpec);
	}
	return METRICS.filter((m) =>
		['success', 'time_s', 'wrong_touch', 'path_eff', 'out_of_bounds', 'score'].includes(m.key)
	);
}

/** Sequential blue ramp, steps 100→700 (dataviz reference palette). */
const RAMP = [
	'#cde2fb', '#b7d3f6', '#9ec5f4', '#86b6ef', '#6da7ec', '#5598e7', '#3987e5',
	'#2a78d6', '#256abf', '#1c5cab', '#184f95', '#104281', '#0d366b'
].map((h) => [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16)));

/** t ∈ [0, 1] → colour; 0 = lightest (least of the metric), 1 = darkest (most). */
export function rampColor(t: number): string {
	const x = Math.max(0, Math.min(1, Number.isFinite(t) ? t : 0)) * (RAMP.length - 1);
	const i = Math.min(RAMP.length - 2, Math.floor(x));
	const f = x - i;
	const c = RAMP[i].map((a, k) => Math.round(a + (RAMP[i + 1][k] - a) * f));
	return `rgb(${c[0]},${c[1]},${c[2]})`;
}

export function metricValue(m: Metrics | Summary, key: string): number | null {
	const v = (m as Record<string, unknown>)[key];
	if (v === null || v === undefined) return null;
	if (typeof v === 'boolean') return v ? 1 : 0;
	return typeof v === 'number' && Number.isFinite(v) ? v : null;
}

export function domainFor(def: MetricDef, values: (number | null)[]): [number, number] {
	if (def.domain) return def.domain;
	const vs = values.filter((v): v is number => v !== null);
	if (!vs.length) return [0, 1];
	const lo = Math.min(...vs);
	const hi = Math.max(...vs);
	return hi - lo < 1e-9 ? [lo, lo + 1] : [lo, hi];
}
