// src/lib/state/simulation.svelte.ts

export interface RewardTerms {
	approach: number;
	ball_to_goal: number;
	possession: number;
	goal: number;
	out_of_bounds: number;
	spin: number;
	time_penalty: number;
	action_magnitude: number;
	[key: string]: number;
}

export interface SimFrame {
	robot_pos: [number, number];
	robot_heading: number;
	ball_pos: [number, number];
	reward_terms: RewardTerms;
	episode: number;
	step: number;
	total_return: number;
}

class SimulationState {
	frame = $state<SimFrame | null>(null);
	connected = $state(false);
	error = $state<string | null>(null);
	episodeReturns = $state<number[]>([]);

	private ws: WebSocket | null = null;
	private _lastEpisode = 0;
	private _url = 'ws://localhost:8765';

	get url() {
		return this._url;
	}

	connect(url?: string) {
		if (url) this._url = url;
		this.disconnect();

		try {
			this.ws = new WebSocket(this._url);

			this.ws.onopen = () => {
				this.connected = true;
				this.error = null;
			};

			this.ws.onclose = () => {
				this.connected = false;
			};

			this.ws.onerror = () => {
				this.error = 'Connection failed';
				this.connected = false;
			};

			this.ws.onmessage = (evt: MessageEvent) => {
				const data = JSON.parse(evt.data as string) as SimFrame;
				if (data.episode !== this._lastEpisode) {
					if (this.frame) {
						this.episodeReturns = [
							...this.episodeReturns.slice(-49),
							this.frame.total_return
						];
					}
					this._lastEpisode = data.episode;
				}
				this.frame = data;
			};
		} catch (e) {
			this.error = String(e);
		}
	}

	disconnect() {
		this.ws?.close();
		this.ws = null;
		this.connected = false;
	}
}

export const simulation = new SimulationState();
