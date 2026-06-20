<script lang="ts">
	import { onMount } from 'svelte';
	import { simulation } from '$lib/state/simulation.svelte.js';
	import LoginControl from '$lib/components/simulation/LoginControl.svelte';
	import ModelBrowser from '$lib/components/overview/ModelBrowser.svelte';
	import ModelCreateForm from '$lib/components/overview/ModelCreateForm.svelte';
	import DevicePanel from '$lib/components/overview/DevicePanel.svelte';
	import { Button } from '$lib/components/ui/button';
	import { Boxes, Plus, Activity, HardDrive } from '@lucide/svelte';

	// The overview panel rides the same public read-only stream as /viz (models,
	// runs, status). Control actions (create/delete/download) need login.
	onMount(() => {
		simulation.connect();
		return () => simulation.disconnect();
	});

	type Tab = 'models' | 'create' | 'devices';
	let tab = $state<Tab>('models');

	const TABS: { value: Tab; label: string; icon: typeof Boxes }[] = [
		{ value: 'models', label: 'Models', icon: Boxes },
		{ value: 'create', label: 'Create', icon: Plus },
		{ value: 'devices', label: 'Devices', icon: HardDrive }
	];

	const conn = $derived.by(() => {
		if (simulation.live) return { label: 'Live', color: '#34d399' };
		if (simulation.connected) return { label: 'Connected', color: '#fbbf24' };
		return { label: 'Reconnecting…', color: '#f87171' };
	});
</script>

<svelte:head>
	<title>Bucky · Overview</title>
</svelte:head>

<div class="min-h-screen bg-background text-foreground">
	<header
		class="flex items-center justify-between border-b border-border/70 px-4 py-2.5 sm:px-6"
	>
		<div class="flex items-center gap-3">
			<h1 class="font-mono text-sm font-semibold uppercase tracking-widest">Bucky Overview</h1>
			<span class="flex items-center gap-1 font-mono text-[11px] text-muted-foreground">
				<span style="color:{conn.color}">●</span>{conn.label}
			</span>
		</div>
		<div class="flex items-center gap-2">
			<Button href="/viz" variant="ghost" size="xs" class="font-mono text-[11px]">
				<Activity class="size-3" />Live viz
			</Button>
			<LoginControl />
		</div>
	</header>

	<div class="mx-auto max-w-5xl px-4 py-5 sm:px-6">
		<div class="mb-5 inline-grid grid-flow-col gap-1 rounded-md border border-border/70 bg-input/20 p-1">
			{#each TABS as t (t.value)}
				<Button
					variant={tab === t.value ? 'default' : 'ghost'}
					size="sm"
					class="h-7 font-mono text-[11px] uppercase tracking-wider"
					onclick={() => (tab = t.value)}
				>
					<t.icon class="size-3.5" />{t.label}
				</Button>
			{/each}
		</div>

		{#if tab === 'models'}
			<ModelBrowser />
		{:else if tab === 'create'}
			<ModelCreateForm onCreated={() => (tab = 'models')} />
		{:else}
			<DevicePanel />
		{/if}
	</div>
</div>
