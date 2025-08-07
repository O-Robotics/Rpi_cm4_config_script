#!/bin/bash
set -e

echo "Setting up Tailscale for auto-start..."

# Check if tailscaled is running
if ! systemctl is-active --quiet tailscaled; then
    echo "Error: tailscaled service is not running."
    echo "Please run 'sudo tailscale up' and verify the connection before executing this script."
    exit 1
fi

# Check if Tailscale has a valid IP address assigned
TAILSCALE_IP=$(tailscale ip -4 2>/dev/null | head -n 1)
if [[ -z "$TAILSCALE_IP" ]]; then
    echo "Error: Tailscale is not connected or no IP assigned."
    echo "Please run 'sudo tailscale up' and verify connection is established."
    exit 1
fi

# Create systemd service to run 'tailscale up' after network is online
cat <<EOF | sudo tee /etc/systemd/system/tailscale-up.service > /dev/null
[Unit]
Description=Tailscale Auto Connect
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/bin/tailscale up
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

# Modify tailscaled.service to wait for network before starting
sudo systemctl edit tailscaled --full --force <<EOF
[Unit]
Description=Tailscale node agent
Documentation=https://tailscale.com/kb/
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/tailscaled
Restart=on-failure
LimitNOFILE=1048576

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd to apply changes
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

# Enable services to run at boot
sudo systemctl enable tailscaled
sudo systemctl enable tailscale-up

# Start services now
sudo systemctl start tailscaled
sudo systemctl start tailscale-up

echo "Tailscale auto-start configuration complete."
