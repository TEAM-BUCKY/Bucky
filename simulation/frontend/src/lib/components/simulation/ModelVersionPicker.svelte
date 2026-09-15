<!--
	Cascading model selector: pick model NAME → VERSION → CHECKPOINT.

	Sources `simulation.models` (each run = one version of a named model) and groups by name the
	same way ModelBrowser does. Resolves to a run + checkpoint, which is exactly the
	{run, checkpoint} PolicyRef the backend match/eval endpoints expect — so callers keep binding
	the same two values. Legacy runs without a version fall back to the run name as the version label.
-->
<script lang="ts">
	import { simulation, type ModelInfo } from '$lib/state/simulation.svelte.js';
	import Combobox from '$lib/components/ui/combobox/Combobox.svelte';

	let {
		run = $bindable(''),
		checkpoint = $bindable(''),
		size = 'sm',
		disabled = false
	}: {
		run?: string;
		checkpoint?: string;
		size?: 'default' | 'sm';
		disabled?: boolean;
	} = $props();

	// Group models by name (each run = one version). Mirrors ModelBrowser's grouping.
	const byName = $derived.by(() => {
		const map = new Map<string, ModelInfo[]>();
		for (const m of simulation.models) {
			const arr = map.get(m.name) ?? [];
			arr.push(m);
			map.set(m.name, arr);
		}
		return map;
	});
	const nameItems = $derived([...byName.keys()].sort().map((n) => ({ value: n, label: n })));

	// The model record for the current run drives the resolved name + available checkpoints.
	const current = $derived(simulation.models.find((m) => m.run === run) ?? null);

	// Selected model name — synced from the run, or set directly by the name combobox.
	let name = $state('');

	// name ← run: an external run (or a version pick) resolves back to its model name.
	$effect(() => {
		if (current && current.name !== name) name = current.name;
	});

	const versions = $derived(name ? (byName.get(name) ?? []) : []);
	const versionItems = $derived(
		versions
			.slice()
			.sort((a, b) => (b.created_at ?? 0) - (a.created_at ?? 0))
			.map((v) => ({ value: v.run, label: v.version ? `v${v.version}` : v.run }))
	);

	// run ← name: picking a new model auto-selects its newest version.
	$effect(() => {
		if (!name) return;
		const vs = byName.get(name);
		if (!vs || vs.length === 0) return;
		if (!vs.some((v) => v.run === run)) {
			const newest = vs.slice().sort((a, b) => (b.created_at ?? 0) - (a.created_at ?? 0))[0];
			run = newest.run;
		}
	});

	const checkpointItems = $derived(
		(current?.checkpoints ?? []).map((c) => ({ value: c.file, label: c.file }))
	);

	// Keep the checkpoint valid when the version changes; prefer best, then final.
	$effect(() => {
		const files = (current?.checkpoints ?? []).map((c) => c.file);
		if (files.length === 0) return;
		if (!files.includes(checkpoint)) {
			checkpoint =
				files.find((f) => f === 'best_model.zip') ??
				files.find((f) => f === 'final_model.zip') ??
				files[0];
		}
	});
</script>

<div class="flex flex-col gap-1.5">
	<Combobox
		bind:value={name}
		items={nameItems}
		{size}
		{disabled}
		placeholder="Model…"
		searchPlaceholder="Search models…"
	/>
	<Combobox
		bind:value={run}
		items={versionItems}
		{size}
		disabled={disabled || !name}
		placeholder="Version…"
		searchPlaceholder="Search versions…"
	/>
	<Combobox
		bind:value={checkpoint}
		items={checkpointItems}
		{size}
		disabled={disabled || !run}
		placeholder="Checkpoint…"
		searchPlaceholder="Search checkpoints…"
	/>
</div>
