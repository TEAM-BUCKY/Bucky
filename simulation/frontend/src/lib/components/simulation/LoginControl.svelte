<script lang="ts">
	import { simulation } from '$lib/state/simulation.svelte.js';
	import * as Dialog from '$lib/components/ui/dialog';
	import { Input } from '$lib/components/ui/input';
	import { Button } from '$lib/components/ui/button';
	import { LogIn, LogOut } from '@lucide/svelte';

	// Viewing is public; logging in unlocks the control actions (launch/stop).
	// Two modes: GitHub-org OAuth (when the server has it configured) or the legacy
	// shared password (the local-dev fallback).
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

{#if simulation.oauthMode}
	<!-- GitHub-org OAuth mode -->
	{#if simulation.authUser}
		<div class="flex items-center gap-1.5">
			{#if simulation.authUser.avatar_url}
				<img
					src={simulation.authUser.avatar_url}
					alt=""
					class="size-4 rounded-full"
				/>
			{/if}
			<span class="font-mono text-[11px] text-muted-foreground">
				<span class="text-emerald-400">●</span>
				{simulation.authUser.login}
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
		<div class="flex items-center gap-1.5">
			{#if simulation.authError}
				<span class="font-mono text-[11px] text-destructive">{simulation.authError}</span>
			{/if}
			<Button
				size="xs"
				variant="outline"
				class="font-mono text-[11px]"
				onclick={() => simulation.loginWithGitHub()}
			>
				<!-- GitHub mark (lucide dropped brand icons in v1, so inline the SVG). -->
				<svg class="size-3" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true">
					<path
						d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.01 8.01 0 0016 8c0-4.42-3.58-8-8-8z"
					/>
				</svg>Sign in with GitHub
			</Button>
		</div>
	{/if}
{:else}
	<!-- Password fallback mode -->
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
{/if}
