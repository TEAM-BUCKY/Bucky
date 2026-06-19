import tailwindcss from '@tailwindcss/vite';
import adapter from '@sveltejs/adapter-static';
import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';

export default defineConfig({
	plugins: [
		tailwindcss(),
		sveltekit({
			compilerOptions: {
				// Force runes mode for the project, except for libraries. Can be removed in svelte 6.
				runes: ({ filename }) => filename.split(/[/\\]/).includes('node_modules') ? undefined : true
			},

			// Static SPA build (no SSR runtime). The fallback page lets client-side
			// routing handle every path; nginx serves the built files behind Traefik.
			adapter: adapter({ fallback: 'index.html' })
		})
	],

	// In dev, proxy `/api` (HTTP + WebSocket) to the backend on :8000 so the app's
	// default same-origin `/api` base "just works" with `pnpm dev` — no VITE_API_BASE
	// or CORS needed, mirroring production where Traefik routes `/api` to the backend.
	server: {
		proxy: {
			'/api': {
				target: 'http://localhost:8000',
				changeOrigin: true,
				ws: true
			}
		}
	}
});
