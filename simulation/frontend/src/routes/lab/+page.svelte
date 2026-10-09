<script lang="ts">
	// Bucky Lab: pick a hand-written module (bucky/lab/user/*.py), sweep it over every start
	// position in the real physics, inspect the heatmap and replay any scenario.
	import { onDestroy, onMount } from 'svelte';
	import { simulation } from '$lib/state/simulation.svelte.js';
	import SoccerField from '$lib/components/SoccerField.svelte';
	import LoginControl from '$lib/components/simulation/LoginControl.svelte';
	import ParamForm from '$lib/components/lab/ParamForm.svelte';
	import SweepSummary from '$lib/components/lab/SweepSummary.svelte';
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import { Label } from '$lib/components/ui/label';
	import { Textarea } from '$lib/components/ui/textarea';
	import { Progress } from '$lib/components/ui/progress';
	import Combobox from '$lib/components/ui/combobox/Combobox.svelte';
	import { ChartLine, FlaskConical, Pause, Play, RefreshCw, RotateCcw } from '@lucide/svelte';
	import {
		defaults,
		labApi,
		toField,
		toFieldTheta,
		acceptsKind,
		type ExperimentInfo,
		type LabModuleInfo,
		type ParamValue,
		type ReplayResult,
		type SensorInfo,
		type SweepJob,
		type SweepRecord,
		type SweepResult
	} from '$lib/lab/api';
	import { METRICS, domainFor, metricDef, metricValue, rampColor } from '$lib/lab/metrics';

	// ── catalogue ────────────────────────────────────────────────────────────
	let modules = $state<LabModuleInfo[]>([]);
	let sensors = $state<SensorInfo[]>([]);
	let experiments = $state<ExperimentInfo[]>([]);
	let loadErrors = $state<Record<string, string>>({});
	let loadError = $state<string | null>(null);

	let moduleKey = $state(''); // "kind/name"
	let params = $state<Record<string, ParamValue>>({});
	let grid = $state<Record<string, ParamValue>>({});
	let sensorParams = $state<Record<string, Record<string, ParamValue>>>({});
	let variantsText = $state('');
	let seed = $state(0);

	const mod = $derived(modules.find((m) => `${m.kind}/${m.name}` === moduleKey));
	const experiment = $derived(experiments.find((e) => acceptsKind(e, mod?.kind)));
	const modSensors = $derived(sensors.filter((s) => mod?.sensors.includes(s.name)));

	async function load() {
		loadError = null;
		try {
			const [m, e] = await Promise.all([labApi.modules(), labApi.experiments()]);
			experiments = e.experiments;
			sensors = m.sensors;
			loadErrors = m.errors;
			modules = m.modules;
			if (!mod && modules.length) selectModule(`${modules[0].kind}/${modules[0].name}`);
			else if (mod) {
				// Reload: keep the user's values but pick up params added to the file.
				params = { ...defaults(mod.params), ...pick(params, mod.params) };
			}
		} catch (err) {
			loadError = (err as Error).message;
		}
	}

	function pick(values: Record<string, ParamValue>, spec: Record<string, unknown>) {
		return Object.fromEntries(Object.entries(values).filter(([k]) => k in spec));
	}

	function selectModule(key: string) {
		moduleKey = key;
		const m = modules.find((x) => `${x.kind}/${x.name}` === key);
		if (!m) return;
		params = defaults(m.params);
		const exp = experiments.find((e) => acceptsKind(e, m.kind));
		grid = exp ? defaults(exp.params) : {};
		sensorParams = Object.fromEntries(
			sensors.filter((s) => m.sensors.includes(s.name)).map((s) => [s.name, defaults(s.params)])
		);
	}

	// ── variants: one per line, "label: key=value, key=value" ────────────────
	function parseVariants(text: string) {
		const lines = text.split('\n').map((l) => l.trim()).filter(Boolean);
		return lines.map((line, i) => {
			const [label, rest] = line.includes(':') ? line.split(/:(.*)/s) : [`v${i + 1}`, line];
			const p: Record<string, ParamValue> = {};
			for (const kv of (rest ?? '').split(',')) {
				const [k, v] = kv.split('=').map((s) => s?.trim());
				if (!k || v === undefined) continue;
				p[k] = v === 'true' ? true : v === 'false' ? false : Number.isFinite(Number(v)) ? Number(v) : v;
			}
			return { label: label.trim() || `v${i + 1}`, params: p };
		});
	}

	// ── sweep job ────────────────────────────────────────────────────────────
	let job = $state<SweepJob | null>(null);
	let result = $state<SweepResult | null>(null);
	let runError = $state<string | null>(null);
	let pollTimer: ReturnType<typeof setTimeout> | null = null;
	const running = $derived(job?.state === 'running');

	async function runSweep() {
		if (!mod || !experiment) return;
		runError = null;
		const variants = parseVariants(variantsText);
		const req = {
			module: mod.name,
			kind: mod.kind,
			experiment: experiment.name,
			params: $state.snapshot(params),
			variants: variants.length ? [{ label: 'base', params: {} }, ...variants] : null,
			grid: $state.snapshot(grid),
			sensor_params: $state.snapshot(sensorParams),
			seed
		};
		try {
			job = await labApi.startSweep(req);
			lastRequest = req;
			poll();
		} catch (err) {
			runError = (err as Error).message;
		}
	}

	let lastRequest: Parameters<typeof labApi.startSweep>[0] | null = null;

	async function poll() {
		if (!job) return;
		try {
			job = await labApi.sweep(job.id);
		} catch (err) {
			runError = (err as Error).message;
			return;
		}
		if (job.state === 'running') {
			pollTimer = setTimeout(poll, 400);
		} else if (job.state === 'done' && job.result) {
			result = job.result;
			variantIdx = 0;
			selectedBall = null;
			replay = null;
		} else if (job.state === 'error') {
			runError = job.error;
		}
	}

	// ── heatmap ──────────────────────────────────────────────────────────────
	let metricKey = $state('success');
	let variantIdx = $state(0);
	let selectedBall = $state<string | null>(null);
	const def = $derived(metricDef(metricKey));
	const variant = $derived(result?.variants[variantIdx] ?? null);
	const fieldMode = $derived(result?.grid.mode === 'field');

	type Cell = { key: string; x: number; y: number; size: number; value: number | null; tip: string; rec?: SweepRecord };

	const cells = $derived.by<Cell[]>(() => {
		if (!variant || !result) return [];
		if (fieldMode) {
			const size = Number(result.grid.robot_step_cm) * 10;
			return variant.records.map((r) => {
				const p = toField(r.scenario.robot);
				const v = metricValue(r.metrics, metricKey);
				return { key: String(r.scenario.id), ...p, size, value: v, rec: r,
					tip: `robot ${(p.x / 10).toFixed(0)}, ${(p.y / 10).toFixed(0)} cm · ${def.label}: ${v === null ? '–' : def.format(v)}` };
			});
		}
		const size = Number(result.grid.ball_step_cm) * 10;
		return (variant.by_ball ?? []).map((row) => {
			const p = toField(row.ball);
			const v = metricValue(row, metricKey);
			return { key: row.ball.join(','), ...p, size, value: v,
				tip: `ball ${(p.x / 10).toFixed(0)}, ${(p.y / 10).toFixed(0)} cm · ${def.label}: ${v === null ? '–' : def.format(v)} (n=${row.n})` };
		});
	});

	const domain = $derived(domainFor(def, cells.map((c) => c.value)));
	const norm = (v: number | null) => (v === null ? null : (v - domain[0]) / (domain[1] - domain[0]));

	// Robot start dots for the selected ball cell (rings mode).
	const dots = $derived.by(() => {
		if (!variant || !selectedBall || fieldMode) return [];
		return variant.records
			.filter((r) => r.scenario.ball.join(',') === selectedBall)
			.map((r) => ({ rec: r, ...toField(r.scenario.robot), value: metricValue(r.metrics, metricKey) }));
	});
	const dotDomain = $derived(domainFor(def, dots.map((d) => d.value)));

	let tip = $state<{ x: number; y: number; text: string } | null>(null);
	let fieldBox = $state<HTMLDivElement | null>(null);
	function showTip(e: MouseEvent, text: string) {
		const r = fieldBox?.getBoundingClientRect();
		if (r) tip = { x: e.clientX - r.left, y: e.clientY - r.top, text };
	}

	function clickCell(c: Cell) {
		if (c.rec) startReplay(c.rec);
		else {
			selectedBall = selectedBall === c.key ? null : c.key;
			replay = null;
		}
	}

	// ── replay ───────────────────────────────────────────────────────────────
	let replay = $state<ReplayResult | null>(null);
	let frame = $state(0);
	let playing = $state(false);
	let speed = $state(0.5);
	let raf = 0;

	async function startReplay(rec: SweepRecord) {
		if (!lastRequest) return;
		runError = null;
		const v = result?.variants.find((x) => x.label === rec.variant);
		if (!fieldMode) selectedBall = rec.scenario.ball.join(',');
		try {
			replay = await labApi.replay({
				module: lastRequest.module,
				kind: lastRequest.kind,
				experiment: lastRequest.experiment,
				params: v ? v.params : lastRequest.params,
				grid: lastRequest.grid,
				sensor_params: lastRequest.sensor_params,
				seed: lastRequest.seed,
				scenario: rec.scenario
			});
			frame = 0;
			play();
		} catch (err) {
			runError = (err as Error).message;
		}
	}

	function play() {
		if (!replay) return;
		if (frame >= replay.trace.length - 1) frame = 0;
		playing = true;
		let last = performance.now();
		let acc = 0;
		cancelAnimationFrame(raf);
		const tick = (now: number) => {
			if (!playing || !replay) return;
			acc += ((now - last) / 1000) * speed;
			last = now;
			const steps = Math.floor(acc / replay.dt);
			if (steps > 0) {
				acc -= steps * replay.dt;
				frame = Math.min(replay.trace.length - 1, frame + steps);
			}
			if (frame >= replay.trace.length - 1) playing = false;
			else raf = requestAnimationFrame(tick);
		};
		raf = requestAnimationFrame(tick);
	}

	function pause() {
		playing = false;
		cancelAnimationFrame(raf);
	}

	const cur = $derived(replay?.trace[Math.min(frame, (replay?.trace.length ?? 1) - 1)] ?? null);
	const trail = $derived(
		replay ? replay.trace.slice(0, frame + 1).map((f) => toField(f.r)).map((p) => `${p.x},${p.y}`).join(' ') : ''
	);
	const markColors: Record<string, string> = { A: '#fab219', C: '#ffffff' };

	const fieldData = $derived.by(() => {
		if (cur) {
			return {
				allies: [{ ...toField(cur.r), theta: toFieldTheta(cur.r[2]) }],
				ball: toField(cur.b),
				showBall: true
			};
		}
		if (fieldMode && result) {
			const b = variant?.records[0]?.scenario.ball;
			return { allies: [], ball: b ? toField(b) : undefined, showBall: !!b };
		}
		if (selectedBall) {
			const [x, y] = selectedBall.split(',').map(Number);
			return { allies: [], ball: toField([x, y]), showBall: true };
		}
		return { allies: [], ball: undefined, showBall: false };
	});

	onMount(() => {
		simulation.connect(); // restores saved credentials + auth mode, like the other pages
		load();
		return () => simulation.disconnect();
	});
	onDestroy(() => {
		if (pollTimer) clearTimeout(pollTimer);
		cancelAnimationFrame(raf);
	});

	const moduleItems = $derived(modules.map((m) => ({ value: `${m.kind}/${m.name}`, label: `${m.name} · ${m.kind}` })));
	const metricItems = METRICS.map((m) => ({ value: m.key, label: m.label }));
	const progress = $derived(job && job.total ? (100 * job.done) / job.total : 0);
</script>

<svelte:head>
	<title>Bucky · Lab</title>
</svelte:head>

<div class="min-h-screen bg-background text-foreground lg:h-screen">
	<div class="flex h-full flex-col gap-4 p-4">
		<Card.Root>
			<Card.Content class="px-4 py-3">
				<div class="flex flex-wrap items-center justify-between gap-3">
					<div class="flex items-baseline gap-3">
						<h1 class="font-mono text-base font-bold tracking-tight text-foreground">BUCKY · LAB</h1>
						<span class="font-mono text-xs text-muted-foreground">
							test hand-written robot code over every start position
						</span>
					</div>
					<div class="flex items-center gap-2">
						<Button href="/viz" variant="ghost" size="xs" class="font-mono text-[11px]">
							<ChartLine class="size-3" />Train
						</Button>
						<Button href="/eval" variant="ghost" size="xs" class="font-mono text-[11px]">
							<FlaskConical class="size-3" />Eval
						</Button>
						<LoginControl />
					</div>
				</div>
			</Card.Content>
		</Card.Root>

		<div class="grid min-h-0 flex-1 grid-cols-1 gap-4 lg:grid-cols-[22rem_minmax(0,1fr)_26rem]">
			<!-- left: module + config -->
			<div class="flex flex-col gap-4 lg:min-h-0 lg:overflow-y-auto [&>*]:shrink-0">
				<Card.Root>
					<Card.Header class="pb-2">
						<div class="flex items-center justify-between">
							<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">Module</Card.Title>
							<Button variant="ghost" size="xs" class="font-mono text-[11px]" onclick={load} title="Re-read bucky/lab/user/*.py">
								<RefreshCw class="size-3" />Reload
							</Button>
						</div>
					</Card.Header>
					<Card.Content class="flex flex-col gap-3">
						<Combobox
							bind:value={() => moduleKey, (v) => selectModule(v)}
							items={moduleItems}
							size="sm"
							disabled={running}
							placeholder="No modules found"
						/>
						{#if mod}
							<p class="font-mono text-[11px] whitespace-pre-line text-muted-foreground">{mod.doc}</p>
							<details class="rounded-md border border-border">
								<summary class="cursor-pointer px-2 py-1 font-mono text-[11px] text-muted-foreground">
									Source · bucky/lab/user/{mod.file}
								</summary>
								<pre class="max-h-80 overflow-auto px-2 pb-2 font-mono text-[10.5px] leading-snug">{mod.source}</pre>
							</details>
							<ParamForm spec={mod.params} bind:values={params} disabled={running} idPrefix="mod" />
						{/if}
						{#if loadError}<p class="font-mono text-[11px] text-red-400">{loadError}</p>{/if}
						{#each Object.entries(loadErrors) as [file, err] (file)}
							<details class="rounded-md border border-red-400/50">
								<summary class="cursor-pointer px-2 py-1 font-mono text-[11px] text-red-400">{file} failed to load</summary>
								<pre class="max-h-60 overflow-auto px-2 pb-2 font-mono text-[10px]">{err}</pre>
							</details>
						{/each}
					</Card.Content>
				</Card.Root>

				{#if experiment}
					<Card.Root>
						<Card.Header class="pb-2">
							<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
								Sweep · {experiment.name}
							</Card.Title>
						</Card.Header>
						<Card.Content class="flex flex-col gap-3">
							<ParamForm spec={experiment.params} bind:values={grid} disabled={running} idPrefix="grid" />
						</Card.Content>
					</Card.Root>
				{/if}

				{#each modSensors as s (s.name)}
					{#if Object.keys(s.params).length && sensorParams[s.name]}
						<Card.Root>
							<Card.Header class="pb-2">
								<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">Sensor · {s.name}</Card.Title>
								<Card.Description class="font-mono text-[11px]">{s.doc}</Card.Description>
							</Card.Header>
							<Card.Content>
								<ParamForm spec={s.params} bind:values={sensorParams[s.name]} disabled={running} idPrefix="sensor-{s.name}" />
							</Card.Content>
						</Card.Root>
					{/if}
				{/each}

				<Card.Root>
					<Card.Content class="flex flex-col gap-3 pt-4">
						<div class="flex flex-col gap-1.5">
							<Label for="lab-variants" class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
								Compare variants (optional)
							</Label>
							<Textarea
								id="lab-variants"
								bind:value={variantsText}
								disabled={running}
								rows={3}
								placeholder={'slow: speed=0.3\nwide: behind_dist=30'}
								class="font-mono text-xs"
							/>
							<span class="font-mono text-[10px] text-muted-foreground/70">
								One per line, "label: key=value, …", applied on top of the params above. "base" is always included.
							</span>
						</div>
						<Button size="sm" class="font-mono text-[11px]" disabled={!mod || running || !simulation.hasCredentials} onclick={runSweep}>
							<Play class="size-3" />{running ? 'Running…' : 'Run sweep'}
						</Button>
						{#if running && job}
							<Progress value={progress} />
							<span class="font-mono text-[11px] text-muted-foreground tabular-nums">{job.done} / {job.total || '…'} episodes</span>
						{/if}
						{#if !simulation.hasCredentials}
							<p class="font-mono text-[11px] text-amber-500">Log in to run sweeps.</p>
						{/if}
						{#if runError}<p class="font-mono text-[11px] whitespace-pre-wrap text-red-400">{runError}</p>{/if}
					</Card.Content>
				</Card.Root>
			</div>

			<!-- center: field -->
			<div class="flex min-w-0 flex-col gap-3 lg:min-h-0">
				<Card.Root class="flex flex-row flex-wrap items-center gap-3 px-3 py-2">
					<div class="w-48"><Combobox bind:value={metricKey} items={metricItems} size="sm" /></div>
					{#if result}
						<div class="flex items-center gap-2 font-mono text-[10px] text-muted-foreground">
							<span>{def.format(domain[0])}</span>
							<span class="h-2.5 w-28 rounded-sm" style="background: linear-gradient(to right, {rampColor(0)}, {rampColor(0.5)}, {rampColor(1)})"></span>
							<span>{def.format(domain[1])}</span>
							<span>· {def.higherIsBetter ? 'higher is better' : 'lower is better'}</span>
						</div>
						<span class="font-mono text-[10px] text-muted-foreground">
							{fieldMode ? 'Click a cell to replay that start' : selectedBall ? 'Dots = robot starts · click one to replay' : 'Click a ball cell to see each start'}
						</span>
					{/if}
				</Card.Root>

				<Card.Root class="relative flex min-h-[28rem] flex-1 items-center justify-center p-3">
					<div class="relative h-full w-full" bind:this={fieldBox} onmouseleave={() => (tip = null)} role="presentation">
						<SoccerField
							fit
							allies={fieldData.allies}
							enemies={[]}
							ball={fieldData.ball}
							showBall={fieldData.showBall}
							showGoalLabels={false}
							class="border-0"
						>
							{#snippet overlay()}
								{#each cells as c (c.key)}
									{@const t = norm(c.value)}
									<rect
										x={c.x - c.size / 2 + 4}
										y={c.y - c.size / 2 + 4}
										width={c.size - 8}
										height={c.size - 8}
										rx="8"
										fill={t === null ? 'transparent' : rampColor(t)}
										fill-opacity={replay ? 0.35 : 1}
										stroke={selectedBall === c.key ? '#fab219' : 'none'}
										stroke-width="14"
										class="cursor-pointer"
										role="button"
										tabindex="-1"
										onclick={() => clickCell(c)}
										onkeydown={() => {}}
										onmousemove={(e) => showTip(e, c.tip)}
									/>
								{/each}
								{#each dots as d (d.rec.scenario.id)}
									{@const t = d.value === null ? null : (d.value - dotDomain[0]) / (dotDomain[1] - dotDomain[0])}
									<circle
										cx={d.x}
										cy={d.y}
										r="30"
										fill={t === null ? '#888' : rampColor(t)}
										stroke="white"
										stroke-width="6"
										class="cursor-pointer"
										role="button"
										tabindex="-1"
										onclick={() => startReplay(d.rec)}
										onkeydown={() => {}}
										onmousemove={(e) =>
											showTip(e, `start ${d.rec.scenario.angle_deg ?? 0}° · ${(d.rec.scenario.radius_cm ?? 0).toFixed(0)} cm · ${def.label}: ${d.value === null ? '–' : def.format(d.value)}`)}
									/>
								{/each}
								{#if replay && cur}
									<polyline points={trail} fill="none" stroke="#fab219" stroke-width="10" stroke-linejoin="round" opacity="0.9" />
									{#each Object.entries(cur.m) as [name, p] (name)}
										{@const f = toField(p)}
										{#if name === 'A'}
											<line x1={toField(cur.r).x} y1={toField(cur.r).y} x2={f.x} y2={f.y} stroke={markColors.A} stroke-width="5" stroke-dasharray="18 12" />
										{/if}
										<circle cx={f.x} cy={f.y} r="22" fill="none" stroke={markColors[name] ?? '#e66767'} stroke-width="8" />
										<circle cx={f.x} cy={f.y} r="6" fill={markColors[name] ?? '#e66767'} />
									{/each}
								{/if}
							{/snippet}
						</SoccerField>
						{#if tip}
							<div
								class="pointer-events-none absolute z-20 rounded-md border border-border bg-popover px-2 py-1 font-mono text-[11px] whitespace-nowrap text-popover-foreground shadow-md"
								style="left: {tip.x + 12}px; top: {tip.y + 12}px"
							>
								{tip.text}
							</div>
						{/if}
					</div>
				</Card.Root>

				{#if replay && cur}
					<Card.Root class="flex flex-row flex-wrap items-center gap-3 px-3 py-2 font-mono text-[11px]">
						{#if playing}
							<Button variant="ghost" size="xs" onclick={pause}><Pause class="size-3" /></Button>
						{:else}
							<Button variant="ghost" size="xs" onclick={play}><Play class="size-3" /></Button>
						{/if}
						<Button variant="ghost" size="xs" onclick={() => { pause(); frame = 0; }}><RotateCcw class="size-3" /></Button>
						<input
							type="range"
							min="0"
							max={replay.trace.length - 1}
							bind:value={frame}
							oninput={pause}
							class="min-w-32 flex-1 accent-primary"
						/>
						<span class="tabular-nums text-muted-foreground">t={cur.t.toFixed(2)} s</span>
						<select bind:value={speed} class="rounded border border-border bg-background px-1 py-0.5">
							{#each [0.1, 0.25, 0.5, 1, 2] as s (s)}<option value={s}>{s}×</option>{/each}
						</select>
						<span class="text-muted-foreground">
							<span style="color: {markColors.A}">●</span> A target
							<span class="ml-2">○</span> C behind
							· {replay.metrics.success ? `behind in ${Number(replay.metrics.time_s).toFixed(2)} s` : 'never got behind'}
							{replay.metrics.wrong_touch ? '· touched ball first' : ''}
							{replay.metrics.out_of_bounds ? '· went out' : ''}
						</span>
						<Button variant="ghost" size="xs" onclick={() => { pause(); replay = null; }}>Close</Button>
					</Card.Root>
				{/if}
			</div>

			<!-- right: results -->
			<div class="flex flex-col gap-4 lg:min-h-0 lg:overflow-y-auto [&>*]:shrink-0">
				{#if result}
					<SweepSummary {result} bind:variantIdx onReplay={startReplay} />
				{:else}
					<Card.Root>
						<Card.Content class="flex flex-col gap-2 pt-4 font-mono text-[11px] text-muted-foreground">
							<p>Add a module by dropping a file in <code>backend/bucky/lab/user/</code>:</p>
							<pre class="overflow-auto rounded-md bg-muted p-2 text-[10.5px] leading-snug">{`from bucky.lab import DriveModule, Param, Vec2, register

@register
class MyDrive(DriveModule):
    """What it does."""
    name = "my_drive"
    params = {"k": Param(1.0, 0, 5, 0.1, "gain")}

    def target(self, ctx):
        B = ctx.ball   # cm, robot at (0,0), +y = enemy goal
        return B - Vec2(0, 20)`}</pre>
							<p>Then press Reload. Runs in the real training physics; results show here.</p>
						</Card.Content>
					</Card.Root>
				{/if}
			</div>
		</div>
	</div>
</div>
