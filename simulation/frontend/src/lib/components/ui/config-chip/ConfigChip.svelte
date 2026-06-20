<script lang="ts">
	import * as Dialog from '$lib/components/ui/dialog';
	import { Button } from '$lib/components/ui/button';
	import { Pencil } from '@lucide/svelte';
	import type { Snippet } from 'svelte';

	let {
		label,
		summary = '',
		invalid = false,
		disabled = false,
		title = '',
		body,
		open = $bindable(false)
	}: {
		/** Short uppercase label shown on the chip (e.g. "Stop"). */
		label: string;
		/** Current value shown next to the label (e.g. "200k steps"). */
		summary?: string;
		/** Tints the chip when the current selection is invalid. */
		invalid?: boolean;
		disabled?: boolean;
		/** Dialog heading; falls back to the label. */
		title?: string;
		/** Dialog body — the actual inputs for this group. */
		body: Snippet;
		open?: boolean;
	} = $props();
</script>

<Dialog.Root bind:open>
	<Dialog.Trigger {disabled}>
		{#snippet child({ props })}
			<button
				{...props}
				type="button"
				{disabled}
				class="group flex w-full items-center gap-2 rounded-md border bg-input/10 px-2.5 py-1.5 text-left shadow-xs transition-colors outline-none hover:bg-input/30 focus-visible:border-ring focus-visible:ring-3 focus-visible:ring-ring/50 disabled:cursor-not-allowed disabled:opacity-50 {invalid
					? 'border-destructive/60'
					: 'border-border/70'}"
			>
				<span class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">
					{label}
				</span>
				<span class="ml-auto truncate font-mono text-[11px] {invalid ? 'text-destructive' : ''}">
					{summary}
				</span>
				<Pencil class="size-3 shrink-0 text-muted-foreground opacity-0 transition-opacity group-hover:opacity-100" />
			</button>
		{/snippet}
	</Dialog.Trigger>
	<Dialog.Content class="gap-4 sm:max-w-md">
		<Dialog.Header>
			<Dialog.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				{title || label}
			</Dialog.Title>
		</Dialog.Header>
		<div class="flex flex-col gap-3">
			{@render body()}
		</div>
		<Dialog.Footer>
			<Button size="sm" class="font-mono text-[11px] uppercase tracking-wider" onclick={() => (open = false)}>
				Done
			</Button>
		</Dialog.Footer>
	</Dialog.Content>
</Dialog.Root>
