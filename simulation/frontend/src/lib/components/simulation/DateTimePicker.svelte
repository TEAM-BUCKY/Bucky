<script lang="ts">
	import Calendar from '$lib/components/ui/calendar/calendar.svelte';
	import * as Popover from '$lib/components/ui/popover/index.js';
	import { Button } from '$lib/components/ui/button/index.js';
	import ChevronDownIcon from '@lucide/svelte/icons/chevron-down';
	import { X } from '@lucide/svelte';
	import { getLocalTimeZone, type DateValue } from '@internationalized/date';

	// `value` is epoch seconds (local wall-clock), or null when nothing is picked.
	let {
		value = $bindable<number | null>(null),
		placeholder = 'Select date'
	}: { value?: number | null; placeholder?: string } = $props();

	let open = $state(false);
	let dateVal = $state<DateValue | undefined>(undefined);
	let timeVal = $state('12:00:00');

	// Let a parent clear the field by setting value back to null.
	$effect(() => {
		if (value === null && dateVal) dateVal = undefined;
	});

	function commit() {
		if (!dateVal) {
			value = null;
			return;
		}
		const [h, m, s] = timeVal.split(':').map((n) => parseInt(n, 10));
		const js = dateVal.toDate(getLocalTimeZone());
		js.setHours(h || 0, m || 0, s || 0, 0);
		value = Math.floor(js.getTime() / 1000);
	}

	function clear() {
		dateVal = undefined;
		value = null;
	}

	// Native <input type="time"> renders as a 12h clock under some browser
	// locales, which makes afternoon times unselectable. Drive a 24h editor off
	// the same `HH:MM:SS` string instead so the format is locale-independent.
	const pad = (n: number) => n.toString().padStart(2, '0');
	const parts = $derived(timeVal.split(':').map((x) => parseInt(x, 10) || 0));

	function setPart(part: 'h' | 'm' | 's', raw: string) {
		const max = part === 'h' ? 23 : 59;
		let n = parseInt(raw, 10);
		if (isNaN(n)) n = 0;
		n = Math.max(0, Math.min(max, n));
		let [h, m, s] = timeVal.split(':').map((x) => parseInt(x, 10) || 0);
		if (part === 'h') h = n;
		else if (part === 'm') m = n;
		else s = n;
		timeVal = `${pad(h)}:${pad(m)}:${pad(s)}`;
		commit();
	}
</script>

<div class="flex gap-2">
	<Popover.Root bind:open>
		<Popover.Trigger>
			{#snippet child({ props })}
				<Button
					{...props}
					variant="outline"
					class="flex-1 justify-between font-mono text-[11px] font-normal {dateVal
						? ''
						: 'text-muted-foreground'}"
				>
					{dateVal ? dateVal.toDate(getLocalTimeZone()).toLocaleDateString() : placeholder}
					<ChevronDownIcon class="size-3.5 opacity-60" />
				</Button>
			{/snippet}
		</Popover.Trigger>
		<Popover.Content class="w-auto overflow-hidden p-0" align="start">
			<Calendar
				type="single"
				bind:value={dateVal}
				onValueChange={() => {
					commit();
					open = false;
				}}
				captionLayout="dropdown"
			/>
			{#if value != null}
				<div class="flex justify-end border-t border-border p-2">
					<Button variant="ghost" size="xs" class="font-mono text-[11px]" onclick={clear}>
						<X class="size-3" />Clear
					</Button>
				</div>
			{/if}
		</Popover.Content>
	</Popover.Root>

	<div
		class="flex w-24 items-center rounded-md border border-input bg-background px-1 font-mono text-[11px]"
	>
		<input
			type="number"
			min="0"
			max="23"
			value={pad(parts[0])}
			onchange={(e) => setPart('h', e.currentTarget.value)}
			aria-label="Hours (24h)"
			class="w-7 bg-transparent text-center outline-none appearance-none [-moz-appearance:textfield] [&::-webkit-inner-spin-button]:appearance-none"
		/>
		<span class="opacity-60">:</span>
		<input
			type="number"
			min="0"
			max="59"
			value={pad(parts[1])}
			onchange={(e) => setPart('m', e.currentTarget.value)}
			aria-label="Minutes"
			class="w-7 bg-transparent text-center outline-none appearance-none [-moz-appearance:textfield] [&::-webkit-inner-spin-button]:appearance-none"
		/>
		<span class="opacity-60">:</span>
		<input
			type="number"
			min="0"
			max="59"
			value={pad(parts[2])}
			onchange={(e) => setPart('s', e.currentTarget.value)}
			aria-label="Seconds"
			class="w-7 bg-transparent text-center outline-none appearance-none [-moz-appearance:textfield] [&::-webkit-inner-spin-button]:appearance-none"
		/>
	</div>
</div>
