<script lang="ts">
	import { Slider as SliderPrimitive } from 'bits-ui';
	import { cn, type WithoutChildrenOrChild } from '$lib/utils.js';

	let {
		ref = $bindable(null),
		value = $bindable(),
		class: className,
		...restProps
	}: WithoutChildrenOrChild<SliderPrimitive.RootProps> = $props();
</script>

<SliderPrimitive.Root
	bind:ref
	bind:value={value as never}
	data-slot="slider"
	class={cn(
		'relative flex w-full touch-none items-center select-none data-disabled:opacity-50',
		className
	)}
	{...restProps}
>
	{#snippet children({ thumbs })}
		<span
			data-slot="slider-track"
			class="bg-muted relative h-1.5 w-full grow overflow-hidden rounded-full"
		>
			<SliderPrimitive.Range data-slot="slider-range" class="bg-primary absolute h-full" />
		</span>
		{#each thumbs as index (index)}
			<SliderPrimitive.Thumb
				{index}
				data-slot="slider-thumb"
				class="border-primary bg-background ring-ring/50 block size-3.5 shrink-0 rounded-full border shadow-sm transition-[color,box-shadow] hover:ring-3 focus-visible:ring-3 focus-visible:outline-hidden disabled:pointer-events-none disabled:opacity-50"
			/>
		{/each}
	{/snippet}
</SliderPrimitive.Root>
