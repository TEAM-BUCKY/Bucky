<script lang="ts">
	import dialogs from '$lib/state/dialog.svelte';
</script>

{#if dialogs.popups.length > 0}
	<div class="fixed inset-0 z-50 flex items-center justify-center">
		<div
			class="absolute inset-0 bg-black/50"
			onclick={() => dialogs.popups.at(-1)?.close()}
		></div>
		{#each dialogs.popups as dialog (dialog.id)}
			<div class="relative z-10">
				<dialog.component {...dialog.data} close={dialog.close} />
			</div>
		{/each}
	</div>
{/if}
