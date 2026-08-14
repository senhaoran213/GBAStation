# GBAStation mGBA Netlink maintenance

## Remotes and branches

- `origin`: `https://github.com/senhaoran213/GBAStation.git` (personal fork).
- `upstream`: `https://github.com/beiklive/GBAStation.git`.
- `main` tracks `upstream/main` only. Sync it with upstream; do not develop on it.
- `codex/mgba-netlink` contains reviewable, mergeable Link Cable changes.
- `codex/test-mgba-netlink-switch` is created from the feature branch and is only for two-Switch test logs, diagnostics, and experimental fixes. Cherry-pick only clean, stable commits back to the feature branch; never submit its temporary debugging content upstream.

## Scope and dependency rules

- The only mGBA Link Cable core source is `https://github.com/senhaoran213/mgba.git`, branch `netlink`.
- `third_party/mgba` is currently vendored source (not a Git submodule). Follow the upstream management approach; do not convert it to a submodule or replace it wholesale in this phase.
- Product scope: two Nintendo Switch consoles on the same Wi-Fi using direct TCP for GBA Link Cable.
- Do not add public-network play, matchmaking, RFU, other-core networking, ROM/BIOS/save distribution, or NSP/forwarder work.

## Change hygiene

- Keep the upstream PR limited to clean, independently reviewable commits from `codex/mgba-netlink`.
- Do not modify the read-only GBAStation reference checkout or the separate `mgba-switch-netlink` protocol/trace repository from this checkout.
