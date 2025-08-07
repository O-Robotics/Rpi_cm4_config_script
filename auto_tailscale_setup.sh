#!/bin/bash
set -e

echo "Setting up Tailscale for auto-start..."

# 1. Create systemd service to run 'tailscale up' after network is online
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

# 2. Modify tailscaled.service to wait for network before starting
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

# 3. Reload systemd to apply changes
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

# 4. Enable services to run at boot
sudo systemctl enable tailscaled
sudo systemctl enable tailscale-up

# 5. Start services immediately (optional)
sudo systemctl start tailscaled
sudo systemctl start tailscale-up

echo "Tailscale auto-start configuration complete."
