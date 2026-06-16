<script lang="ts">
	import * as Popover from '$lib/components/ui/popover';
	import * as Command from '$lib/components/ui/command';
	import { cn } from '$lib/utils.js';
	import ChevronsUpDownIcon from '@lucide/svelte/icons/chevrons-up-down';

	let {
		value = $bindable(''),
		items,
		placeholder = 'Select…',
		searchPlaceholder = 'Search…',
		empty = 'Nothing found.',
		disabled = false,
		size = 'default',
		class: className = ''
	}: {
		value?: string;
		items: { value: string; label: string; disabled?: boolean }[];
		placeholder?: string;
		searchPlaceholder?: string;
		empty?: string;
		disabled?: boolean;
		size?: 'default' | 'sm';
		class?: string;
	} = $props();

	let open = $state(false);

	const selectedLabel = $derived(items.find((i) => i.value === value)?.label ?? placeholder);
</script>

<Popover.Root bind:open>
	<Popover.Trigger
		{disabled}
		class={cn(
			'border-input flex w-full items-center justify-between rounded-md border bg-transparent px-2.5 text-left shadow-xs transition-colors outline-none focus-visible:border-ring focus-visible:ring-3 focus-visible:ring-ring/50 disabled:cursor-not-allowed disabled:opacity-50',
			size === 'sm' ? 'h-8 text-xs' : 'h-9 text-sm',
			!value && 'text-muted-foreground',
			className
		)}
	>
		<span class="min-w-0 flex-1 truncate font-mono">{selectedLabel}</span>
		<ChevronsUpDownIcon class="ml-2 size-4 shrink-0 text-muted-foreground" />
	</Popover.Trigger>
	<Popover.Content
		align="start"
		sideOffset={4}
		class="w-[var(--bits-popover-anchor-width)] p-0"
	>
		<Command.Root>
			<Command.Input placeholder={searchPlaceholder} />
			<Command.List>
				<Command.Empty>{empty}</Command.Empty>
				{#each items as item}
					<Command.Item
						value={item.label}
						keywords={[item.value]}
						disabled={item.disabled}
						onSelect={() => {
							value = item.value;
							open = false;
						}}
					>
						{item.label}
					</Command.Item>
				{/each}
			</Command.List>
		</Command.Root>
	</Popover.Content>
</Popover.Root>
