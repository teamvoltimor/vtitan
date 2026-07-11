FROM node:22-alpine AS builder

WORKDIR /app

RUN corepack enable pnpm

# pnpm-workspace.yaml must be present at install time -- it holds the
# allowBuilds approval for esbuild's postinstall script; without it pnpm
# refuses to run that script and the install fails.
COPY package.json pnpm-lock.yaml pnpm-workspace.yaml ./
RUN --mount=type=cache,target=/root/.pnpm-store \
    pnpm install --frozen-lockfile

COPY . .
RUN pnpm run build

# nginx-unprivileged: non-root by default, listens on 8080 out of the box
# (no custom nginx.conf needed — its stock default.conf already binds 8080).
FROM nginxinc/nginx-unprivileged:1.27-alpine AS runtime

COPY --from=builder /app/dist /usr/share/nginx/html

EXPOSE 8080

HEALTHCHECK --interval=10s --timeout=5s --retries=5 --start-period=5s \
    CMD wget --spider -q http://localhost:8080/ || exit 1

CMD ["nginx", "-g", "daemon off;"]
