# Shared SSH preflight for the Pi 5 helper scripts. Source, do not execute.
#
# Reaching the Pi is less reliable than it looks:
#
#   * Two ssh binaries exist on a Windows dev box -- Git Bash's /usr/bin/ssh and
#     Windows OpenSSH in System32 -- and which one wins depends on the shell the
#     task was launched from. They do not share an agent.
#   * There may be no ssh-agent at all, in which case auth comes from the
#     IdentityFile in ~/.ssh/config and BatchMode=yes forbids the prompt that
#     would otherwise cover a passphrase.
#   * rpi-5-local resolves over mDNS (.local), which intermittently fails with
#     "Could not resolve hostname" and succeeds on the next attempt.
#
# So: retry, fall back to allowing a prompt, and report what ssh actually said
# instead of a flat "cannot reach".

PI5_SSH_ATTEMPTS="${PI5_SSH_ATTEMPTS:-3}"

# ssh_preflight <host> [ssh opts...]
#
# Set SSH_PREFLIGHT_INTERACTIVE=0 before calling to skip the prompt fallback --
# correct for scripts that run unattended on the robot, where a password prompt
# would hang instead of failing. Set SSH_PREFLIGHT_HINT to replace the generic
# advice with something specific to that link.
ssh_preflight() {
  local host="$1"
  shift
  local -a opts=("$@")
  local attempt output

  for ((attempt = 1; attempt <= PI5_SSH_ATTEMPTS; attempt++)); do
    if output="$(ssh "${opts[@]}" -o BatchMode=yes "$host" "echo ok" 2>&1)"; then
      return 0
    fi
    [ "$attempt" -lt "$PI5_SSH_ATTEMPTS" ] && sleep 2
  done

  if [ "${SSH_PREFLIGHT_INTERACTIVE:-1}" = "1" ]; then
    # Key auth alone did not work. Try once more allowing a prompt, which covers
    # a passphrase-protected key with no agent.
    echo "[ssh] key-only auth failed for $host; retrying interactively..." >&2
    echo "[ssh] last error: ${output:-<none>}" >&2
    if ssh "${opts[@]}" "$host" "echo ok" >/dev/null 2>&1; then
      return 0
    fi
  fi

  {
    echo "[ssh] ERROR: cannot reach $host over SSH."
    echo
    echo "  last error: ${output:-<none>}"
    echo "  ssh in use: $(command -v ssh)"
    echo "  resolves to: $(ssh -G "$host" 2>/dev/null | awk '/^hostname /{print $2}')"
    echo
    if [ -n "${SSH_PREFLIGHT_HINT:-}" ]; then
      echo "$SSH_PREFLIGHT_HINT"
    else
      cat <<'EOF'
Things worth trying, in order:
  1. ssh <host>                    -- does it work by hand from this same shell?
  2. PI5_HOST=user@<ip> task ...   -- bypasses .local mDNS, which is flaky
  3. If "resolves to" above shows the alias rather than a real hostname, your
     ~/.ssh/config was not read. On Windows that usually means the script ran
     under WSL's bash (C:\Windows\System32\bash.exe) instead of Git Bash: WSL
     has its own $HOME with no config, and cannot resolve .local at all.
  4. eval $(ssh-agent) && ssh-add <your key>  -- if the key has a passphrase
EOF
    fi
  } >&2
  return 1
}

# Back-compat name for the Pi 5 scripts.
pi5_preflight() { ssh_preflight "$@"; }
