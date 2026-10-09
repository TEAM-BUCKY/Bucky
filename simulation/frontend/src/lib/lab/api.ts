// Bucky Lab client: types + fetch helpers for /api/lab/* (see backend/bucky/lab).
import { apiBases, simulation } from '$lib/state/simulation.svelte.js';

export type ParamValue = number | boolean | string | number[];

export interface ParamSpec {
	type: 'float' | 'int' | 'bool' | 'str' | 'list' | 'choice';
	default: ParamValue;
	min: number | null;
	max: number | null;
	step: number | null;
	help: string;
	options: string[] | null;
}

export interface LabModuleInfo {
	kind: string;
	name: string;
	doc: string;
	params: Record<string, ParamSpec>;
	sensors: string[];
	file: string | null;
	source: string;
}

export interface SensorInfo {
	name: string;
	doc: string;
	params: Record<string, ParamSpec>;
}

export interface ExperimentInfo {
	name: string;
	module_kind: string;
	/** module_kind plus kinds that can stand in for it (e.g. "firmware" programs). */
	accepts_kinds?: string[];
	doc: string;
	params: Record<string, ParamSpec>;
}

export interface Scenario {
	id: number;
	ball: [number, number]; // sim frame, metres
	robot: [number, number];
	heading: number;
	radius_cm?: number;
	angle_deg?: number;
}

export type Metrics = Record<string, number | boolean | null>;

export interface SweepRecord {
	variant: string;
	scenario: Scenario;
	metrics: Metrics;
}

export type Summary = Record<string, number> & { n: number };

export interface VariantResult {
	label: string;
	params: Record<string, ParamValue>;
	records: SweepRecord[];
	overall: Summary;
	worst: SweepRecord[];
	by_ball?: (Summary & { ball: [number, number] })[];
	by_angle?: (Summary & { angle_deg: number })[];
}

export interface SweepResult {
	module: string;
	kind: string;
	experiment: string;
	grid: Record<string, ParamValue>;
	n_scenarios: number;
	elapsed_s: number;
	variants: VariantResult[];
}

export interface SweepJob {
	id: string;
	state: 'running' | 'done' | 'error';
	done: number;
	total: number;
	error: string | null;
	result?: SweepResult;
}

export interface TraceFrame {
	t: number;
	r: [number, number, number]; // x, y, heading
	b: [number, number];
	m: Record<string, [number, number]>; // debug marks, sim frame
}

export interface ReplayResult {
	scenario: Scenario;
	dt: number;
	metrics: Metrics;
	trace: TraceFrame[];
}

export interface SweepRequest {
	module: string;
	kind: string;
	experiment: string;
	params: Record<string, ParamValue>;
	variants?: { label: string; params: Record<string, ParamValue> }[] | null;
	grid: Record<string, ParamValue>;
	sensor_params: Record<string, Record<string, ParamValue>>;
	seed: number;
}

async function errorText(res: Response): Promise<string> {
	try {
		const j = await res.json();
		if (j?.detail) return typeof j.detail === 'string' ? j.detail : JSON.stringify(j.detail);
	} catch {
		/* non-JSON body */
	}
	return `Request failed (${res.status})`;
}

async function getJson<T>(path: string): Promise<T> {
	const res = await fetch(apiBases().httpBase + path);
	if (!res.ok) throw new Error(await errorText(res));
	return res.json();
}

async function postJson<T>(path: string, body: unknown): Promise<T> {
	const res = await fetch(apiBases().httpBase + path, {
		method: 'POST',
		...simulation.authInit({ 'Content-Type': 'application/json' }),
		body: JSON.stringify(body)
	});
	if (res.status === 401) throw new Error('Log in to run lab sweeps.');
	if (!res.ok) throw new Error(await errorText(res));
	return res.json();
}

export const labApi = {
	modules: (kind?: string) =>
		getJson<{ modules: LabModuleInfo[]; errors: Record<string, string>; sensors: SensorInfo[] }>(
			'/lab/modules' + (kind ? `?kind=${encodeURIComponent(kind)}` : '')
		),
	experiments: () => getJson<{ experiments: ExperimentInfo[] }>('/lab/experiments'),
	startSweep: (req: SweepRequest) => postJson<SweepJob>('/lab/sweep', req),
	sweep: (id: string) => getJson<SweepJob>(`/lab/sweep/${encodeURIComponent(id)}`),
	replay: (req: Omit<SweepRequest, 'variants'> & { scenario: Scenario }) =>
		postJson<ReplayResult>('/lab/replay', req)
};

/** Sim frame (m, +x = enemy goal) → SoccerField space (mm, +y = enemy goal, +x = right).
 * Matches the user frame the modules are written in (same picture as the GeoGebra sketch). */
export function toField(p: readonly [number, number] | readonly number[]): { x: number; y: number } {
	return { x: -p[1] * 1000, y: p[0] * 1000 };
}

/** Sim heading → SoccerField theta (π/2 = facing the enemy goal). */
export function toFieldTheta(h: number): number {
	return h + Math.PI / 2;
}

export function defaults(spec: Record<string, ParamSpec>): Record<string, ParamValue> {
	return Object.fromEntries(Object.entries(spec).map(([k, p]) => [k, Array.isArray(p.default) ? [...p.default] : p.default]));
}

/** True when `exp` can test modules of `kind`. */
export function acceptsKind(exp: ExperimentInfo, kind: string | undefined): boolean {
	if (!kind) return false;
	return exp.module_kind === kind || (exp.accepts_kinds ?? []).includes(kind);
}
