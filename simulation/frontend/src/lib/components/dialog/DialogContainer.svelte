<script lang="ts">
	import dialogs from '$lib/state/dialog.svelte.js';
</script>

<!--
	Each dialog body renders its own bits-ui <Dialog.Root>, which portals an overlay + content to
	<body> and owns dimming, focus-trap, Escape and outside-click dismissal. We pass each popup its
	`close` callback so the dialog can route bits-ui's onOpenChange back to DialogsState. We do NOT
	render a backdrop here — doing so would double-dim and sit beneath the portal, unable to receive
	clicks (the original dismissal bug).
-->
{#each dialogs.popups as dialog (dialog.id)}
	{@const Popup = dialog.component}
	{#if Popup}
		<Popup {...dialog.data} close={dialog.close} />
	{/if}
{/each}
