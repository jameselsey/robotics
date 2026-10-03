#!/usr/bin/env bash
set -euo pipefail
# Adapt the installed vendor supervisor config; do not change host permissions.
sed -i 's/user=iot-user/user=root/g' /etc/supervisor/conf.d/supervisord.conf
mkdir -p /usr/share/qcom/conf.d
cp /etc/fastrpc/hexagon-dsp-binaries.yaml /usr/share/qcom/conf.d/
exec /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf
