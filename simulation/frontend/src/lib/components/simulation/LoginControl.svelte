<script lang="ts">
	import { simulation } from '$lib/state/simulation.svelte.js';
	import * as Dialog from '$lib/components/ui/dialog';
	import { Input } from '$lib/components/ui/input';
	import { Button } from '$lib/components/ui/button';
	import { LogIn, LogOut } from '@lucide/svelte';

	// Viewing is public; logging in unlocks the control actions (launch/stop).
	let open = $state(false);
	let username = $state('');
	let password = $state('');

	function submit(e: Event) {
		e.preventDefault();
		if (!username || !password) return;
		simulation.setCredentials(username, password);
		password = '';
		open = false;
	}
</script>

{#if simulation.username}
	<div class="flex items-center gap-1.5">
		{#if simulation.authError}
			<span class="font-mono text-[11px] text-destructive">{simulation.authError}</span>
		{/if}
		<span class="font-mono text-[11px] text-muted-foreground">
			<span class="text-emerald-400">●</span> {simulation.username}
		</span>
		<Button
			variant="ghost"
			size="xs"
			class="font-mono text-[11px]"
			onclick={() => simulation.logout()}
		>
			<LogOut class="size-3" />Log out
		</Button>
	</div>
{:else}
	<Dialog.Root bind:open>
		<Dialog.Trigger>
			{#snippet child({ props })}
				<Button {...props} size="xs" variant="outline" class="font-mono text-[11px]">
					<LogIn class="size-3" />Log in
				</Button>
			{/snippet}
		</Dialog.Trigger>
		<Dialog.Content class="sm:max-w-sm">
			<Dialog.Header>
				<Dialog.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
					Log in to control
				</Dialog.Title>
				<Dialog.Description class="font-mono text-[11px]">
					Viewing is public. Enter the control credentials to launch or stop jobs.
				</Dialog.Description>
			</Dialog.Header>

			<form class="flex flex-col gap-3" onsubmit={submit}>
				<div class="flex flex-col gap-1.5">
					<label for="login-username" class="font-mono text-[11px] text-muted-foreground">
						Username
					</label>
					<Input
						id="login-username"
						type="text"
						bind:value={username}
						placeholder="user"
						autocomplete="username"
						spellcheck={false}
						class="font-mono text-xs"
					/>
				</div>
				<div class="flex flex-col gap-1.5">
					<label for="login-password" class="font-mono text-[11px] text-muted-foreground">
						Password
					</label>
					<Input
						id="login-password"
						type="password"
						bind:value={password}
						placeholder="password"
						autocomplete="current-password"
						class="font-mono text-xs"
					/>
				</div>

				{#if simulation.authError}
					<span class="font-mono text-[11px] text-destructive">{simulation.authError}</span>
				{/if}

				<Dialog.Footer>
					<Button type="submit" size="sm" class="font-mono text-xs" disabled={!username || !password}>
						<LogIn class="size-3.5" />Log in
					</Button>
				</Dialog.Footer>
			</form>
		</Dialog.Content>
	</Dialog.Root>
{/if}
