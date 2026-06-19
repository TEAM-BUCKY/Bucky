<script lang="ts">
    import * as Dialog from "$lib/components/ui/dialog";
    import * as Card from '$lib/components/ui/card';

    import {simulation} from "$lib/state/simulation.svelte.ts";
    import LineChart from "$lib/components/LineChart.svelte";

    const CHARTS: { key: string; label: string; color: string }[] = [
        { key: 'rollout/ep_rew_mean', label: 'ep reward mean', color: '#34d399' },
        { key: 'train/loss', label: 'loss', color: '#f87171' },
        { key: 'train/approx_kl', label: 'approx kl', color: '#fbbf24' },
        { key: 'train/entropy_loss', label: 'entropy', color: '#a78bfa' },
        { key: 'train/value_loss', label: 'value loss', color: '#22d3ee' },
        { key: 'train/policy_gradient_loss', label: 'pg loss', color: '#5a8cff' },
        { key: 'train/clip_fraction', label: 'clip frac', color: '#f0abfc' },
        { key: 'rollout/ep_len_mean', label: 'ep len mean', color: '#94a3b8' }
    ];
    const activeCharts = $derived(CHARTS.filter((c) => (simulation.metrics[c.key]?.length ?? 0) > 0));
</script>

<Dialog.Root class="w-[50vw]" open={true}>
    <Dialog.Content class="w-[50vw]">
        <Dialog.Header>
            <Dialog.Title class="font-mono text-xs font-semibold uppercase tracking-widest">
                Training scalars
            </Dialog.Title>
        </Dialog.Header>
        {#if activeCharts.length}
            <div class="grid grid-cols-2 gap-2.5 md:grid-cols-3 xl:grid-cols-4">
                {#each activeCharts as c (c.key)}
                    <LineChart label={c.label} points={simulation.metrics[c.key]} color={c.color} />
                {/each}
            </div>
        {:else}
            <p class="font-mono text-xs text-muted-foreground">
                Scalars appear after the first PPO update completes.
            </p>
        {/if}
    </Dialog.Content>
</Dialog.Root>