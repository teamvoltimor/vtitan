# 0067. The board link is an NM-only USB-gadget subnet and the Zero receives built tarballs

- Status: accepted
- Date: 2026-09-15

## Context

Two hardware constraints shaped the deployment. The Pi Zero 2 W has about 415 MB
of usable RAM, so resolving the full pixi environment or rebuilding `ros2_ws` on
it swap-thrashes the SD card into double-digit load averages, makes SSH
unresponsive for minutes, and has crashed the board with an unclean shutdown. And
the link between the boards sits on a range that a VPN was intercepting.

## Options considered

- (a) Keep the direct link on `10.0.0.0/8` and let netplan manage the gadget
      interface; build on the Zero.
- (b) Move the gadget link to a VPN-safe range, run it NetworkManager-only, and
      build on the Pi 5 and ship tarballs to the Zero.

## Decision

(b). The Pi Zero to Pi 5 USB-Ethernet gadget is `192.168.250.0/24`, Zero `.1` and
Pi 5 `.2`, managed by NetworkManager only: no netplan, no manual `ip` service, no
ICS switcher. The gadget boots with `dtoverlay=dwc2,dr_mode=peripheral` and
`modules-load=dwc2,g_ether`, and fixed MACs in `/etc/modprobe.d/g_ether.conf`, so
NetworkManager can bind a static profile on a point-to-point link with no DHCP.
The `netplan-eth0` profile's autoconnect is disabled because it matched any
ethernet-class device including `usb0`, and a udev override clears NetworkManager's
stock NM_UNMANAGED flag for USB gadget devices.

Builds run on the Pi 5 (15 GB+ RAM, under a minute) and are copied to the Zero as
tarballs of `.pixi/envs/dev` and the `ros2_ws` build; both boards are
`linux-aarch64` with the same username and identical absolute repo path, so no
conda or mamba resolution happens on the Zero. Provisioning is Ansible (roles
`common`, `pi5`, `pi_zero`).

## Consequences

- The Zero never resolves or builds; a maintenance window must still disable
  `vtitan-pi-zero.service`, because `pixi run` silently self-repairs a stale env
  in the background.
- The link must carry multicast routes (`routes4: [224.0.0.0/4]`) or DDS discovery
  breaks.

## History

- d3026d6c 2026-06-23: move the direct link off `192.168.1.x` to `10.250.250.0/24`.
- c252c43c, 619fb4c5, c862497d 2026-07-06: migrate off `10.250.250.0/24` to
  `192.168.250.0/24`; the VPN was intercepting all `10.0.0.0/8` and silently
  blackholing the link.
- 5e527075 2026-07-07: purge the last `10.250.250.x` stragglers.
- ec46f2ab 2026-07-07: split the Windows-to-Pi5 direct link to `192.168.251.0/24`,
  keeping the gadget at `.250`.
- 3293b896 2026-07-21: add build-on-Pi5-ship-to-Zero.
- 80227b74 2026-07-25: make the gadget reproducible (dwc2, fixed MACs, static
  profile). Without fixed MACs the gadget gets a random address each boot and
  NetworkManager never binds the static profile.
- ac6530e0 and dbaa9a9c 2026-07-29/30: introduce Ansible and delete the manual
  scp and ssh provisioning scripts.
- cecb1ed1 2026-08-01: stop building the Zero's workspace on the Zero.
- 91ad9574 2026-08-09: stop netplan's generated ethernet profile from claiming
  `usb0`, the fix that made the link NM-only.
- 907d526b 2026-08-20: restore the missing `usb0` udev override, the real root
  cause (NetworkManager's stock rule ignores every USB gadget device).
- 74a42bc4 2026-08-10: remove the manual gadget task, which lacked
  `routes4: [224.0.0.0/4]` and would have reintroduced the DDS discovery bug.

## Cross-references

- 0065 owns the UDP-only transport that crosses this link.

## Evidence

- Never run `pixi shell`, `install` or `run` on the Pi Zero: it rebuilds the
  shipped environment, crashes mid-install, and can leave the card in fsck. The
  Zero workspace is built on the Pi 5 and shipped.
