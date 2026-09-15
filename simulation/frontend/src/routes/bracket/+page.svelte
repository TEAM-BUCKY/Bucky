<script lang="ts">
	import { onMount } from 'svelte';
	import { simulation, type ModelInfo } from '$lib/state/simulation.svelte.js';
	import LoginControl from '$lib/components/simulation/LoginControl.svelte';
	import ModelVersionPicker from '$lib/components/simulation/ModelVersionPicker.svelte';
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import { Input } from '$lib/components/ui/input';
	import { Label } from '$lib/components/ui/label';
	import { Checkbox } from '$lib/components/ui/checkbox';
	import { Trophy, Plus, Cpu, Play, Square, X, Boxes, Layers } from '@lucide/svelte';

	onMount(() => {
		simulation.connect();
		return () => simulation.disconnect();
	});

	// One bracket entrant. run='classical' selects the hand-coded controller (no checkpoint).
	interface Entrant {
		run: string;
		checkpoint: string;
		label: string;
	}
	let entrants = $state<Entrant[]>([]);
	let matchesPerPairing = $state(5);
	let seed = $state(0);
	let quick = $state(true);

	const t = $derived(simulation.tournament);
	const running = $derived(t?.status === 'running');
	const standings = $derived(t?.standings?.standings ?? []);
	const canLaunch = $derived(
		simulation.hasCredentials && !running && validEntrants().length >= 2
	);

	function validEntrants(): Entrant[] {
		return entrants.filter((e) => e.run === 'classical' || (e.run && e.checkpoint));
	}
	function addModel() {
		entrants.push({ run: '', checkpoint: '', label: '' });
	}
	function addClassical() {
		entrants.push({ run: 'classical', checkpoint: '', label: 'classical' });
	}
	function remove(i: number) {
		entrants.splice(i, 1);
	}

	function pickCheckpoint(m: ModelInfo): string {
		const files = m.checkpoints.map((c) => c.file);
		return (
			files.find((f) => f === 'best_model.zip') ??
			files.find((f) => f === 'final_model.zip') ??
			files.find((f) => f.endsWith('.zip')) ??
			files[0] ??
			''
		);
	}

	/** One-click: enter the newest version of every saved model (plus the classical controller). */
	function addAllModels() {
		const byName = new Map<string, ModelInfo[]>();
		for (const m of simulation.models) {
			const arr = byName.get(m.name) ?? [];
			arr.push(m);
			byName.set(m.name, arr);
		}
		const next: Entrant[] = [{ run: 'classical', checkpoint: '', label: 'classical' }];
		for (const [name, versions] of byName) {
			const newest = versions.slice().sort((a, b) => (b.created_at ?? 0) - (a.created_at ?? 0))[0];
			const ckpt = pickCheckpoint(newest);
			if (ckpt) next.push({ run: newest.run, checkpoint: ckpt, label: name });
		}
		entrants = next;
	}

	async function launch() {
		await simulation.launchTournament({
			entrants: validEntrants().map((e) => ({
				run: e.run,
				checkpoint: e.run === 'classical' ? '' : e.checkpoint,
				label: e.label.trim() || undefined
			})),
			matchesPerPairing: Math.max(1, Number(matchesPerPairing) || 1),
			seed: Number(seed) || 0,
			// Quick = cap each match short (faster, noisier); full = a complete 2×7-min match.
			maxSteps: quick ? 800 : 200000
		});
	}

	const conn = $derived.by(() => {
		if (simulation.live) return { label: 'Live', color: '#34d399' };
		if (simulation.connected) return { label: 'Connected', color: '#fbbf24' };
		return { label: 'Reconnecting…', color: '#f87171' };
	});

	const statusColor: Record<string, string> = {
		running: '#fbbf24',
		done: '#34d399',
		stopped: '#94a3b8',
		error: '#f87171'
	};

	const COLS = [
		['P', 'played'],
		['W', 'wins'],
		['D', 'draws'],
		['L', 'losses'],
		['GF', 'goals_for'],
		['GA', 'goals_against'],
		['GD', 'goal_diff'],
		['Pts', 'points'],
		['Elo', 'elo'],
		['K/m', 'kicks_per_match']
	] as const;
</script>

<svelte:head>
	<title>Bucky · Competition</title>
</svelte:head>

<div class="min-h-screen bg-background text-foreground">
	<header class="flex items-center justify-between border-b border-border/70 px-4 py-2.5 sm:px-6">
		<div class="flex items-center gap-3">
			<h1 class="flex items-center gap-2 font-mono text-sm font-semibold uppercase tracking-widest">
				<Trophy class="size-4" />Competition
			</h1>
			<span class="flex items-center gap-1 font-mono text-[11px] text-muted-foreground">
				<span style="color:{conn.color}">●</span>{conn.label}
			</span>
		</div>
		<div class="flex items-center gap-2">
			<Button href="/overview" variant="ghost" size="xs" class="font-mono text-[11px]">
				<Boxes class="size-3" />Models
			</Button>
			<LoginControl />
		</div>
	</header>

	<div class="mx-auto grid max-w-6xl gap-5 px-4 py-5 sm:px-6 lg:grid-cols-[minmax(0,26rem)_1fr]">
		<!-- ── Setup ─────────────────────────────────────────────────────── -->
		<Card.Root class="h-fit">
			<Card.Header class="pb-2">
				<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
					Entrants
				</Card.Title>
			</Card.Header>
			<Card.Content class="flex flex-col gap-3">
				{#each entrants as entrant, i (i)}
					<div class="rounded-md border border-border/70 bg-input/10 p-2">
						<div class="mb-1.5 flex items-center gap-2">
							<span class="font-mono text-[10px] uppercase text-muted-foreground">#{i + 1}</span>
							{#if entrant.run === 'classical'}
								<span class="flex items-center gap-1 font-mono text-[11px]">
									<Cpu class="size-3" />classical controller
								</span>
							{/if}
							<button
								class="ml-auto text-muted-foreground hover:text-destructive"
								title="Remove entrant"
								onclick={() => remove(i)}
							>
								<X class="size-3.5" />
							</button>
						</div>
						{#if entrant.run !== 'classical'}
							<ModelVersionPicker
								bind:run={entrant.run}
								bind:checkpoint={entrant.checkpoint}
								size="sm"
								disabled={running}
							/>
						{/if}
						<Input
							bind:value={entrant.label}
							placeholder="Label (optional)"
							disabled={running}
							class="mt-1.5 h-8 font-mono text-xs"
						/>
					</div>
				{/each}

				<div class="flex gap-2">
					<Button
						variant="outline"
						size="sm"
						class="flex-1 font-mono text-[11px]"
						disabled={running}
						onclick={addModel}
					>
						<Plus class="size-3" />Model
					</Button>
					<Button
						variant="outline"
						size="sm"
						class="flex-1 font-mono text-[11px]"
						disabled={running}
						onclick={addClassical}
					>
						<Cpu class="size-3" />Classical
					</Button>
				</div>
				<Button
					variant="secondary"
					size="sm"
					class="font-mono text-[11px]"
					disabled={running || simulation.models.length === 0}
					title="Enter the newest version of every saved model, plus the classical controller"
					onclick={addAllModels}
				>
					<Layers class="size-3" />All models ({simulation.models.length})
				</Button>

				<div class="flex gap-3">
					<div class="flex flex-1 flex-col gap-1.5">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
							Matches / pair
						</Label>
						<Input
							type="number"
							min="1"
							bind:value={matchesPerPairing}
							disabled={running}
							class="h-8 font-mono text-xs tabular-nums"
						/>
					</div>
					<div class="flex flex-1 flex-col gap-1.5">
						<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
							Seed
						</Label>
						<Input
							type="number"
							bind:value={seed}
							disabled={running}
							class="h-8 font-mono text-xs tabular-nums"
						/>
					</div>
				</div>

				<label class="flex items-center gap-2">
					<Checkbox bind:checked={quick} disabled={running} />
					<span class="font-mono text-[11px] text-muted-foreground">
						Quick matches (faster, noisier)
					</span>
				</label>

				{#if running}
					<Button
						variant="destructive"
						size="sm"
						class="font-mono text-[11px]"
						onclick={() => t && simulation.stopTournament(t.id)}
					>
						<Square class="size-3" />Stop after current match
					</Button>
				{:else}
					<Button size="sm" class="font-mono text-[11px]" disabled={!canLaunch} onclick={launch}>
						<Play class="size-3" />Run round-robin ({validEntrants().length} entrants)
					</Button>
				{/if}

				{#if !simulation.hasCredentials}
					<p class="font-mono text-[11px] text-amber-500">Log in to run a competition.</p>
				{/if}
			</Card.Content>
		</Card.Root>

		<!-- ── Standings ─────────────────────────────────────────────────── -->
		<Card.Root class="h-fit">
			<Card.Header class="pb-2">
				<div class="flex items-center gap-2">
					<Card.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
						Standings
					</Card.Title>
					{#if t}
						<span
							class="font-mono text-[10px] uppercase"
							style="color:{statusColor[t.status] ?? '#94a3b8'}"
						>
							● {t.status}
						</span>
						{#if t.progress && t.progress.total > 0}
							<span class="ml-auto font-mono text-[10px] text-muted-foreground">
								match {t.progress.match}/{t.progress.total}
								{#if t.progress.pairing}· {t.progress.pairing}{/if}
							</span>
						{/if}
					{/if}
				</div>
			</Card.Header>
			<Card.Content>
				{#if standings.length === 0}
					<p class="py-10 text-center font-mono text-xs text-muted-foreground">
						{t ? 'Waiting for the first match…' : 'Add entrants and run a round-robin to see a ranking.'}
					</p>
				{:else}
					<div class="overflow-x-auto">
						<table class="w-full border-collapse font-mono text-xs tabular-nums">
							<thead>
								<tr class="border-b border-border/70 text-[10px] uppercase text-muted-foreground">
									<th class="px-2 py-1.5 text-left">#</th>
									<th class="px-2 py-1.5 text-left">Model</th>
									{#each COLS as [head] (head)}
										<th class="px-2 py-1.5 text-right">{head}</th>
									{/each}
								</tr>
							</thead>
							<tbody>
								{#each standings as s, rank (s.label)}
									<tr class="border-b border-border/40 {rank === 0 ? 'bg-emerald-500/5' : ''}">
										<td class="px-2 py-1.5 text-left">{rank + 1}</td>
										<td class="px-2 py-1.5 text-left font-semibold">{s.label}</td>
										{#each COLS as [head, key] (head)}
											<td class="px-2 py-1.5 text-right {head === 'Pts' ? 'font-semibold' : ''}">
												{s[key]}
											</td>
										{/each}
									</tr>
								{/each}
							</tbody>
						</table>
					</div>
					{#if t?.standings && !t.standings.complete && t.status !== 'running'}
						<p class="mt-2 font-mono text-[10px] text-muted-foreground">
							Stopped early — partial standings.
						</p>
					{/if}
				{/if}
			</Card.Content>
		</Card.Root>
	</div>
</div>
