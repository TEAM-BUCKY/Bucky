<script lang="ts">
	import { tick } from 'svelte';
	import SoccerField from '$lib/components/SoccerField.svelte';
	import * as Card from '$lib/components/ui/card';
	import { Button } from '$lib/components/ui/button';
	import { Input } from '$lib/components/ui/input';
	import { robotLink, fwToField, ballInField } from '$lib/state/robotLink.svelte';
	import { Usb, Bluetooth, Unplug, Download, Trash2, ChartLine, TestTubes, Boxes } from '@lucide/svelte';

	const link = robotLink;
	const s = $derived(link.state);

	// Robot pose in the firmware field frame (cm, cw from +y). Without an EKF pose the robot is drawn at
	// the centre, turned by the compass offset, so the ball still shows where the robot sees it.
	const robot = $derived.by(() => {
		if (s?.pose) return { x: s.pose.x, y: s.pose.y, theta: s.pose.theta };
		return { x: 0, y: 0, theta: ((s?.offset ?? 0) * Math.PI) / 180 };
	});
	const field = $derived.by(() => {
		const r = fwToField(robot.x, robot.y, robot.theta);
		const ball = s?.ball?.seen ? ballInField(robot, s.ball) : null;
		return { allies: [r], ball: ball ? fwToField(ball.x, ball.y) : undefined };
	});

	const conn = $derived.by(() => {
		if (link.connected) return { label: link.transport === 'usb' ? 'USB' : 'Bluetooth', color: '#34d399' };
		if (link.connecting) return { label: 'Connecting…', color: '#fbbf24' };
		return { label: 'Disconnected', color: '#f87171' };
	});

	// Bars scale to the largest value in the current frame (IR is 12-bit ADC, line is ambient-corrected).
	function bars(values: number[] | undefined) {
		if (!values) return [];
		const max = Math.max(1, ...values.map(Math.abs));
		return values.map((v) => ({ v, pct: (Math.abs(v) / max) * 100 }));
	}
	const irBars = $derived(bars(s?.ir));
	const lineBars = $derived(bars(s?.line));

	let filter = $state('');
	let autoscroll = $state(true);
	let logBox = $state<HTMLDivElement>();
	const shownLogs = $derived.by(() => {
		const f = filter.trim().toLowerCase();
		return f ? link.logs.filter((l) => l.text.toLowerCase().includes(f)) : link.logs;
	});
	$effect(() => {
		void shownLogs;
		if (!autoscroll || !logBox) return;
		const box = logBox;
		void tick().then(() => (box.scrollTop = box.scrollHeight));
	});

	const fmt = (v: number | null | undefined, d = 1) => (v === null || v === undefined ? '–' : v.toFixed(d));
	const fmtMs = (ms: number | null) => (ms === null ? '' : (ms / 1000).toFixed(2).padStart(8, ' '));
</script>

<svelte:head>
	<title>Bucky · Robot</title>
</svelte:head>

<div class="h-screen bg-background text-foreground">
	<div class="flex h-full flex-col gap-4 p-4">
		<Card.Root>
			<Card.Content class="px-4 py-3">
				<div class="flex flex-wrap items-center justify-between gap-3">
					<div class="flex items-baseline gap-3">
						<h1 class="font-mono text-base font-bold tracking-tight text-foreground">BUCKY · ROBOT</h1>
						<span class="font-mono text-xs text-muted-foreground">
							{#if link.hello}{link.hello.board} · protocol v{link.hello.v}{:else}live telemetry over USB or Bluetooth{/if}
							{#if link.connected}<span class="text-foreground/70"> · {link.msgRate} msg/s</span>{/if}
						</span>
					</div>
					<div class="flex flex-wrap items-center gap-2">
						<span class="flex items-center gap-1.5 rounded-full border border-border bg-background/60 px-2.5 py-1">
							<span class="size-2 rounded-full" style="background: {conn.color}"></span>
							<span class="font-mono text-[11px] text-muted-foreground">{conn.label}</span>
						</span>
						{#if link.connected || link.connecting}
							<Button variant="outline" size="xs" class="font-mono text-[11px]" onclick={() => link.disconnect()}>
								<Unplug class="size-3" />Disconnect
							</Button>
						{:else}
							<Button
								variant="outline"
								size="xs"
								class="font-mono text-[11px]"
								disabled={!link.serialSupported}
								title={link.serialSupported ? 'Connect over the USB cable' : 'Web Serial needs Chrome or Edge'}
								onclick={() => link.connectSerial()}
							>
								<Usb class="size-3" />USB
							</Button>
							<Button
								variant="outline"
								size="xs"
								class="font-mono text-[11px]"
								disabled={!link.bluetoothSupported}
								title={link.bluetoothSupported
									? 'Connect to the HM-10 (disconnect any phone first)'
									: 'Web Bluetooth needs Chrome or Edge'}
								onclick={() => link.connectBluetooth()}
							>
								<Bluetooth class="size-3" />Bluetooth
							</Button>
						{/if}
						<Button href="/viz" variant="ghost" size="xs" class="font-mono text-[11px]">
							<ChartLine class="size-3" />Train
						</Button>
						<Button href="/lab" variant="ghost" size="xs" class="font-mono text-[11px]">
							<TestTubes class="size-3" />Lab
						</Button>
						<Button href="/overview" variant="ghost" size="xs" class="font-mono text-[11px]">
							<Boxes class="size-3" />Overview
						</Button>
					</div>
				</div>
				{#if link.error}
					<p class="mt-2 font-mono text-[11px] text-destructive">{link.error}</p>
				{/if}
			</Card.Content>
		</Card.Root>

		<div class="grid min-h-0 flex-1 grid-cols-1 gap-4 lg:grid-cols-[18rem_minmax(0,1fr)_18rem]">
			<!-- left: scalar sensors -->
			<div class="flex flex-col gap-4 lg:min-h-0 lg:overflow-y-auto">
				<Card.Root>
					<Card.Header><Card.Title class="font-mono text-xs">Compass</Card.Title></Card.Header>
					<Card.Content class="grid grid-cols-2 gap-2 font-mono text-xs">
						<span class="text-muted-foreground">heading</span><span class="tabular-nums">{fmt(s?.heading)}°</span>
						<span class="text-muted-foreground">offset</span><span class="tabular-nums">{fmt(s?.offset)}°</span>
					</Card.Content>
				</Card.Root>
				<Card.Root>
					<Card.Header><Card.Title class="font-mono text-xs">Ball (IR)</Card.Title></Card.Header>
					<Card.Content class="grid grid-cols-2 gap-2 font-mono text-xs">
						<span class="text-muted-foreground">seen</span><span>{s?.ball ? (s.ball.seen ? 'yes' : 'no') : '–'}</span>
						<span class="text-muted-foreground">angle</span>
						<span class="tabular-nums">{s?.ball ? fmt((s.ball.angle * 180) / Math.PI) : '–'}°</span>
						<span class="text-muted-foreground">distance</span><span class="tabular-nums">{fmt(s?.ball?.dist)} cm</span>
						<span class="text-muted-foreground">confidence</span><span class="tabular-nums">{fmt(s?.ball?.conf, 2)}</span>
					</Card.Content>
				</Card.Root>
				<Card.Root>
					<Card.Header><Card.Title class="font-mono text-xs">Sonar</Card.Title></Card.Header>
					<Card.Content class="grid grid-cols-2 gap-2 font-mono text-xs">
						{#each s?.sonar ?? [] as d, i (i)}
							<span class="text-muted-foreground">S{i}</span><span class="tabular-nums">{d === null ? 'timeout' : `${fmt(d)} cm`}</span>
						{:else}
							<span class="col-span-2 text-muted-foreground">no data</span>
						{/each}
					</Card.Content>
				</Card.Root>
				<Card.Root>
					<Card.Header><Card.Title class="font-mono text-xs">Encoders</Card.Title></Card.Header>
					<Card.Content class="grid grid-cols-2 gap-2 font-mono text-xs">
						{#each s?.enc ?? [] as v, i (i)}
							<span class="text-muted-foreground">M{i + 1}</span><span class="tabular-nums">{fmt(v, 2)}</span>
						{:else}
							<span class="col-span-2 text-muted-foreground">no data</span>
						{/each}
					</Card.Content>
				</Card.Root>
				{#if s?.pose}
					<Card.Root>
						<Card.Header><Card.Title class="font-mono text-xs">Pose (EKF)</Card.Title></Card.Header>
						<Card.Content class="grid grid-cols-2 gap-2 font-mono text-xs">
							<span class="text-muted-foreground">x, y</span><span class="tabular-nums">{fmt(s.pose.x)}, {fmt(s.pose.y)} cm</span>
							<span class="text-muted-foreground">θ</span><span class="tabular-nums">{fmt((s.pose.theta * 180) / Math.PI)}°</span>
							<span class="text-muted-foreground">v</span><span class="tabular-nums">{fmt(s.pose.vx)}, {fmt(s.pose.vy)} cm/s</span>
						</Card.Content>
					</Card.Root>
				{/if}
			</div>

			<!-- centre: field -->
			<Card.Root class="mx-auto flex aspect-[303/242] w-full max-w-2xl shrink-0 items-center justify-center p-3">
				<SoccerField
					fit
					allies={field.allies}
					enemies={[]}
					ball={field.ball}
					showBall={!!field.ball}
					rotation={90}
					class="border-0"
				/>
			</Card.Root>

			<!-- right: sensor arrays -->
			<div class="flex flex-col gap-4 lg:min-h-0 lg:overflow-y-auto">
				<Card.Root>
					<Card.Header><Card.Title class="font-mono text-xs">IR ring</Card.Title></Card.Header>
					<Card.Content class="flex flex-col gap-1 font-mono text-[11px]">
						{#each irBars as b, i (i)}
							<div class="flex items-center gap-2">
								<span class="w-5 text-muted-foreground">{i}</span>
								<div class="h-2 flex-1 rounded bg-muted">
									<div class="h-2 rounded bg-orange-400" style="width: {b.pct}%"></div>
								</div>
								<span class="w-10 text-right tabular-nums">{b.v}</span>
							</div>
						{:else}
							<span class="text-muted-foreground">no data</span>
						{/each}
					</Card.Content>
				</Card.Root>
				<Card.Root>
					<Card.Header><Card.Title class="font-mono text-xs">Line (green, reflected)</Card.Title></Card.Header>
					<Card.Content class="flex flex-col gap-1 font-mono text-[11px]">
						{#each lineBars as b, i (i)}
							<div class="flex items-center gap-2">
								<span class="w-5 text-muted-foreground">{i}</span>
								<div class="h-2 flex-1 rounded bg-muted">
									<div class="h-2 rounded bg-emerald-400" style="width: {b.pct}%"></div>
								</div>
								<span class="w-10 text-right tabular-nums">{b.v}</span>
							</div>
						{:else}
							<span class="text-muted-foreground">no data</span>
						{/each}
					</Card.Content>
				</Card.Root>
			</div>
		</div>

		<!-- logs -->
		<Card.Root class="flex h-64 shrink-0 flex-col">
			<Card.Content class="flex min-h-0 flex-1 flex-col gap-2 px-4 py-3">
				<div class="flex flex-wrap items-center gap-2">
					<span class="font-mono text-xs font-semibold">Logs</span>
					<Input bind:value={filter} placeholder="filter…" class="h-7 w-48 font-mono text-[11px]" />
					<label class="flex items-center gap-1.5 font-mono text-[11px] text-muted-foreground">
						<input type="checkbox" bind:checked={autoscroll} />autoscroll
					</label>
					<span class="ml-auto font-mono text-[11px] text-muted-foreground">
						{link.recorded} recorded · {link.missedFrames} missed frames · {s?.drop ?? 0} dropped on robot
						{#if link.parseErrors > 0} · {link.parseErrors} bad lines{/if}
					</span>
					<Button variant="ghost" size="xs" class="font-mono text-[11px]" onclick={() => link.clearLogs()}>
						<Trash2 class="size-3" />Clear
					</Button>
					<Button
						variant="ghost"
						size="xs"
						class="font-mono text-[11px]"
						disabled={link.recorded === 0}
						onclick={() => link.downloadJsonl()}
					>
						<Download class="size-3" />.jsonl
					</Button>
				</div>
				<div bind:this={logBox} class="min-h-0 flex-1 overflow-y-auto rounded border border-border bg-background/60 p-2">
					<pre class="font-mono text-[11px] leading-snug">{#each shownLogs as l (l.id)}<span
								class="text-muted-foreground">{fmtMs(l.ms)}</span> <span class={l.raw ? 'text-amber-400' : ''}>{l.text}</span>
{/each}</pre>
				</div>
			</Card.Content>
		</Card.Root>
	</div>
</div>
