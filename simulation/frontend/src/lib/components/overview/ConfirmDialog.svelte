<script lang="ts">
	import * as Dialog from '$lib/components/ui/dialog';
	import { Button } from '$lib/components/ui/button';

	// A reusable confirmation dialog (replaces native window.confirm). The parent
	// binds `open` and supplies the copy + the action to run on confirm.
	let {
		open = $bindable(false),
		title = 'Are you sure?',
		description = '',
		confirmLabel = 'Confirm',
		cancelLabel = 'Cancel',
		destructive = false,
		onconfirm = () => {}
	}: {
		open?: boolean;
		title?: string;
		description?: string;
		confirmLabel?: string;
		cancelLabel?: string;
		destructive?: boolean;
		onconfirm?: () => void;
	} = $props();

	function handleConfirm() {
		open = false;
		onconfirm();
	}
</script>

<Dialog.Root bind:open>
	<Dialog.Content class="sm:max-w-sm">
		<Dialog.Header>
			<Dialog.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
				{title}
			</Dialog.Title>
			{#if description}
				<Dialog.Description class="font-mono text-[11px] leading-relaxed">
					{description}
				</Dialog.Description>
			{/if}
		</Dialog.Header>
		<Dialog.Footer>
			<Button variant="ghost" size="sm" class="font-mono text-xs" onclick={() => (open = false)}>
				{cancelLabel}
			</Button>
			<Button
				variant={destructive ? 'destructive' : 'default'}
				size="sm"
				class="font-mono text-xs"
				onclick={handleConfirm}
			>
				{confirmLabel}
			</Button>
		</Dialog.Footer>
	</Dialog.Content>
</Dialog.Root>
