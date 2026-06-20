<script lang="ts">
	import { simulation } from '$lib/state/simulation.svelte.js';
	import * as Card from '$lib/components/ui/card';
	import * as Dialog from '$lib/components/ui/dialog';
	import { Input } from '$lib/components/ui/input';
	import { Label } from '$lib/components/ui/label';
	import { Button } from '$lib/components/ui/button';
	import ConfirmDialog from './ConfirmDialog.svelte';
	import { HardDrive, Plus, Trash2, Copy, Check, Terminal } from '@lucide/svelte';

	const canControl = $derived(simulation.hasCredentials);

	let newName = $state('');
	let issued = $state<{ name: string; token: string } | null>(null);
	let copied = $state(false);

	// Revoke confirmation dialog state.
	let confirmOpen = $state(false);
	let pendingRevoke = $state<{ id: string; name: string } | null>(null);

	// "Show command" dialog state.
	let cmdOpen = $state(false);
	let cmdLoading = $state(false);
	let cmdDevice = $state('');
	let cmdToken = $state('');
	let cmdCopied = $state(false);

	const serverUrl = $derived(simulation.workerServerUrl);
	const cmd = $derived(
		`uv run python scripts/worker.py --server ${serverUrl} --token ${cmdToken}`
	);

	// Rotating the token (old one stops working) is the only way to obtain a usable
	// command after registration — tokens are never recoverable once shown.
	async function showCommand(id: string, name: string) {
		cmdDevice = name;
		cmdToken = '';
		cmdCopied = false;
		cmdLoading = true;
		cmdOpen = true;
		cmdToken = (await simulation.rotateDeviceToken(id)) ?? '';
		cmdLoading = false;
	}

	async function copyCommand() {
		try {
			await navigator.clipboard.writeText(cmd);
			cmdCopied = true;
		} catch {
			/* clipboard unavailable */
		}
	}

	async function register() {
		const name = newName.trim();
		if (!name) return;
		const token = await simulation.registerDevice(name);
		if (token) {
			issued = { name, token };
			copied = false;
			newName = '';
		}
	}

	async function copyToken() {
		if (!issued) return;
		try {
			await navigator.clipboard.writeText(issued.token);
			copied = true;
		} catch {
			/* clipboard unavailable */
		}
	}

	function revoke(id: string, name: string) {
		pendingRevoke = { id, name };
		confirmOpen = true;
	}

	function fmtSeen(epoch: number | null): string {
		if (!epoch) return 'never';
		const secs = Math.max(0, Math.floor(Date.now() / 1000 - epoch));
		if (secs < 60) return `${secs}s ago`;
		if (secs < 3600) return `${Math.floor(secs / 60)}m ago`;
		return new Date(epoch * 1000).toLocaleString();
	}
</script>

<div class="flex flex-col gap-4">
	<!-- Register a device -->
	<Card.Root class="max-w-2xl">
		<Card.Header class="pb-2 pt-3">
			<Card.Title class="flex items-center gap-2 font-mono text-xs font-semibold uppercase tracking-widest">
				<Plus class="size-4" />Register a guest device
			</Card.Title>
		</Card.Header>
		<Card.Content class="flex flex-col gap-3">
			<div class="flex items-end gap-2">
				<div class="flex flex-1 flex-col gap-1">
					<Label class="font-mono text-[10px] uppercase tracking-wider text-muted-foreground">Device name</Label>
					<Input
						bind:value={newName}
						class="font-mono"
						disabled={!canControl}
						onkeydown={(e: KeyboardEvent) => e.key === 'Enter' && register()}
					/>
				</div>
				<Button disabled={!canControl || !newName.trim()} onclick={register}>
					<Plus class="size-4" />Register
				</Button>
			</div>
			{#if !canControl}
				<p class="font-mono text-[11px] text-muted-foreground">Log in (top right) to register devices.</p>
			{/if}

			{#if issued}
				<div class="rounded-md border border-emerald-500/40 bg-emerald-500/5 p-3">
					<p class="mb-1 font-mono text-[11px] text-emerald-400">
						Token for "{issued.name}":
					</p>
					<div class="flex items-center gap-2">
						<code class="flex-1 overflow-x-auto rounded bg-background/60 px-2 py-1 font-mono text-[11px]">
							{issued.token}
						</code>
						<Button variant="outline" size="xs" class="font-mono text-[10px]" onclick={copyToken}>
							{#if copied}<Check class="size-3" />Copied{:else}<Copy class="size-3" />Copy{/if}
						</Button>
						<Button
							variant="ghost"
							size="xs"
							class="font-mono text-[10px]"
							onclick={() => (issued = null)}
						>
							Dismiss
						</Button>
					</div>
				</div>
			{/if}
		</Card.Content>
	</Card.Root>

	<!-- Device list -->
	<Card.Root>
		<Card.Header class="pb-2 pt-3">
			<Card.Title class="flex items-center gap-2 font-mono text-xs font-semibold uppercase tracking-widest">
				<HardDrive class="size-4" />Devices
				<span class="font-mono text-[10px] text-muted-foreground">{simulation.devices.length}</span>
			</Card.Title>
		</Card.Header>
		<Card.Content class="flex flex-col gap-2">
			{#if simulation.devices.length === 0}
				<p class="py-6 text-center font-mono text-xs text-muted-foreground">
					No devices registered yet.
				</p>
			{/if}
			{#each simulation.devices as d (d.id)}
				<div class="flex items-center gap-2 rounded-md border border-border/70 bg-input/10 px-3 py-2">
					<span
						class="size-2 shrink-0 rounded-full {d.online ? 'animate-pulse' : ''}"
						style="background:{d.online ? '#34d399' : '#64748b'}"
						title={d.online ? 'online' : 'offline'}
					></span>
					<span class="font-mono text-xs font-semibold">{d.name}</span>
					{#if d.current_job}
						<span class="font-mono text-[10px] text-amber-400">▶ {d.current_job}</span>
					{/if}
					<span class="ml-auto flex items-center gap-3 font-mono text-[10px] text-muted-foreground">
						<span>seen {fmtSeen(d.last_seen)}</span>
						<Button
							variant="ghost"
							size="xs"
							class="font-mono text-[10px]"
							disabled={!canControl}
							onclick={() => showCommand(d.id, d.name)}
						>
							<Terminal class="size-3" />Show command
						</Button>
						<Button
							variant="ghost"
							size="xs"
							class="font-mono text-[10px] text-destructive hover:text-destructive"
							disabled={!canControl}
							onclick={() => revoke(d.id, d.name)}
						>
							<Trash2 class="size-3" />Revoke
						</Button>
					</span>
				</div>
			{/each}
		</Card.Content>
	</Card.Root>
</div>

<ConfirmDialog
	bind:open={confirmOpen}
	title="Revoke device"
	description={pendingRevoke
		? `Revoke device "${pendingRevoke.name}"? Its token stops working immediately.`
		: ''}
	confirmLabel="Revoke"
	destructive
	onconfirm={() => pendingRevoke && simulation.revokeDevice(pendingRevoke.id)}
/>

<Dialog.Root bind:open={cmdOpen}>
	<Dialog.Content class="gap-4 sm:max-w-xl">
		<Dialog.Header>
			<Dialog.Title class="flex items-center gap-2 font-mono text-xs font-semibold uppercase tracking-widest">
				<Terminal class="size-4" />Connect "{cmdDevice}"
			</Dialog.Title>
			<Dialog.Description class="font-mono text-[11px]">
				Run this on the device to lease &amp; train jobs from the server's queue.
			</Dialog.Description>
		</Dialog.Header>

		{#if cmdLoading}
			<p class="font-mono text-[11px] text-muted-foreground">Issuing a fresh token…</p>
		{:else if !cmdToken}
			<p class="font-mono text-[11px] text-destructive">
				Could not issue a token. Make sure you're logged in and try again.
			</p>
		{:else}
			<div class="flex items-start gap-2">
				<code class="flex-1 overflow-x-auto rounded bg-background/60 px-2 py-2 font-mono text-[11px] whitespace-pre-wrap break-all">{cmd}</code>
				<Button variant="outline" size="xs" class="shrink-0 font-mono text-[10px]" onclick={copyCommand}>
					{#if cmdCopied}<Check class="size-3" />Copied{:else}<Copy class="size-3" />Copy{/if}
				</Button>
			</div>
			<p class="font-mono text-[11px] text-muted-foreground">
				Or set <code class="text-foreground">BUCKY_SERVER</code> /
				<code class="text-foreground">BUCKY_DEVICE_TOKEN</code> and run
				<code class="text-foreground">scripts/worker.py</code> with no flags.
			</p>
			<p class="font-mono text-[10px] text-amber-400">
				This rotated the device's token, any previous command for "{cmdDevice}" no longer works!
			</p>
		{/if}

		<Dialog.Footer>
			<Button size="sm" class="font-mono text-[11px] uppercase tracking-wider" onclick={() => (cmdOpen = false)}>
				Done
			</Button>
		</Dialog.Footer>
	</Dialog.Content>
</Dialog.Root>
