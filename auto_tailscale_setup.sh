#!/bin/bash
set -e

echo "Setting up Tailscale auto-start (non-destructive)..."

# Check tailscale connection status
if ! systemctl is-active --quiet tailscaled; then
    echo "Warning: tailscaled is not running. Please connect with 'sudo tailscale up' before proceeding."
    exit 1
fi

if [[ -z $(tailscale ip -4 2>/dev/null | head -n 1) ]]; then
    echo "Warning: Tailscale is running but not connected. Please check 'sudo tailscale up'."
    exit 1
fi

# Step 1: Create systemd override safely
sudo mkdir -p /etc/systemd/system/tailscaled.service.d

cat <<EOF | sudo tee /etc/systemd/system/tailscaled.service.d/override.conf > /dev/null
[Unit]
After=network-online.target
Wants=network-online.target
EOF

# Step 2: Create tailscale-up.service to re-run 'tailscale up' at boot
cat <<EOF | sudo tee /etc/systemd/system/tailscale-up.service > /dev/null
[Unit]
Description=Tailscale Auto Connect
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/bin/tailscale up
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF

# Step 3: Reload and enable
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable tailscaled
sudo systemctl enable tailscale-up

echo "Tailscale auto-start configuration complete. You can now reboot to verify."
