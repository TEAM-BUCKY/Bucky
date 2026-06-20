# Frontend: build the static SPA, then serve it with nginx.
# Build on Debian (glibc), NOT alpine: Vite 8 / rolldown and lightningcss load native
# binaries, and pnpm --frozen-lockfile can fail to resolve the musl variant on alpine,
# making `pnpm build` die with exit code 1. glibc uses the proven *-gnu bindings.
FROM node:22-slim AS build
WORKDIR /app
RUN corepack enable

COPY package.json pnpm-lock.yaml pnpm-workspace.yaml .npmrc ./
RUN --mount=type=cache,target=/root/.local/share/pnpm/store \
    pnpm install --frozen-lockfile

COPY . .
# Same-origin '/api' by default (Traefik routes /api to the backend on the same host).
ARG VITE_API_BASE=/api
ENV VITE_API_BASE=$VITE_API_BASE
RUN pnpm build

FROM nginx:alpine AS run
COPY --from=build /app/build /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
