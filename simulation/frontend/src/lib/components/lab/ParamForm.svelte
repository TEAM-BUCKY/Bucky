<script lang="ts">
	// Renders a form straight from a module/sensor/experiment `params` spec (bucky.lab.params.Param).
	// With `allowSearch`, number fields also take "*" / "*lo..hi" and choices/bools a "*" option:
	// the Lab then searches that param for the best fit (backend bucky/lab/optimize.py).
	import { Input } from '$lib/components/ui/input';
	import { Label } from '$lib/components/ui/label';
	import { Checkbox } from '$lib/components/ui/checkbox';
	import Combobox from '$lib/components/ui/combobox/Combobox.svelte';
	import { isStar, type ParamSpec, type ParamValue } from '$lib/lab/api';

	let {
		spec,
		values = $bindable(),
		disabled = false,
		idPrefix,
		allowSearch = false
	}: {
		spec: Record<string, ParamSpec>;
		values: Record<string, ParamValue>;
		disabled?: boolean;
		idPrefix: string;
		allowSearch?: boolean;
	} = $props();

	const STAR_ITEM = { value: '*', label: '* (search)' };

	// A spec can render a frame before its values are filled in (e.g. right after switching modules).
	const val = (name: string) => values[name] ?? spec[name].default;

	function setNumber(name: string, raw: string) {
		if (allowSearch && raw.trim().startsWith('*')) {
			values[name] = raw.trim();
			return;
		}
		const v = Number(raw);
		if (raw.trim() !== '' && Number.isFinite(v)) values[name] = v;
	}

	function setList(name: string, raw: string) {
		const vs = raw
			.split(/[,;\s]+/)
			.filter(Boolean)
			.map(Number);
		if (vs.every(Number.isFinite)) values[name] = vs;
	}
</script>

<div class="grid grid-cols-2 gap-x-3 gap-y-2.5">
	{#each Object.entries(spec) as [name, p] (name)}
		{@const id = `${idPrefix}-${name}`}
		<div class="flex flex-col gap-1 {p.type === 'list' || p.type === 'choice' ? 'col-span-2' : ''}" title={p.help}>
			{#if p.type === 'bool'}
				<div class="flex h-full items-end gap-2 pb-1.5">
					{#if isStar(val(name))}
						<span class="font-mono text-[11px] text-primary">*</span>
					{:else}
						<Checkbox {id} checked={!!val(name)} onCheckedChange={(c) => (values[name] = !!c)} {disabled} />
					{/if}
					<Label for={id} class="font-mono text-[11px] text-muted-foreground">{name}</Label>
					{#if allowSearch}
						<button
							type="button"
							class="ml-auto font-mono text-[10px] {isStar(val(name)) ? 'text-primary' : 'text-muted-foreground/60 hover:text-foreground'}"
							title="Search"
							{disabled}
							onclick={() => (values[name] = isStar(val(name)) ? Boolean(p.default) : '*')}
						>*</button>
					{/if}
				</div>
			{:else}
				<Label for={id} class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">{name}</Label>
				{#if p.type === 'choice'}
					<Combobox
						bind:value={() => String(val(name)), (v) => (values[name] = v)}
						items={[...(p.options ?? []).map((o) => ({ value: o, label: o })), ...(allowSearch ? [STAR_ITEM] : [])]}
						size="sm"
						{disabled}
					/>
				{:else if p.type === 'list'}
					<Input
						{id}
						value={(val(name) as number[]).join(', ')}
						onchange={(e) => setList(name, e.currentTarget.value)}
						{disabled}
						class="h-8 font-mono text-xs tabular-nums"
					/>
				{:else if p.type === 'str'}
					<Input
						{id}
						value={String(val(name))}
						onchange={(e) => (values[name] = e.currentTarget.value)}
						{disabled}
						class="h-8 font-mono text-xs"
					/>
				{:else if allowSearch}
					<!-- Text, not type=number, so it can hold "*" / "*lo..hi". -->
					<Input
						{id}
						inputmode="decimal"
						value={String(val(name))}
						onchange={(e) => setNumber(name, e.currentTarget.value)}
						{disabled}
						placeholder={p.min !== null && p.max !== null ? `${p.min}–${p.max}` : undefined}
						class="h-8 font-mono text-xs tabular-nums {isStar(val(name)) ? 'border-primary text-primary' : ''}"
					/>
				{:else}
					<Input
						{id}
						type="number"
						min={p.min ?? undefined}
						max={p.max ?? undefined}
						step={p.step ?? (p.type === 'int' ? 1 : 'any')}
						value={val(name) as number}
						onchange={(e) => setNumber(name, e.currentTarget.value)}
						{disabled}
						class="h-8 font-mono text-xs tabular-nums"
					/>
				{/if}
			{/if}
		</div>
	{/each}
</div>
