#!/bin/bash
# =====================================================================================
# setup_hotspot.sh ― broon 휴대폰 무선 조종용 핫스팟 '처음 한 번' 설정  (2026-11-23, 코드만 — 아직 돌려 보지 않았다)
# =====================================================================================
#   sudo bash ~/hl_ws/src/broon/hotspot/setup_hotspot.sh        ← ★사용자가 직접★ (joy 노드는 sudo 비밀번호를 칠 수 없다)
#
# 하는 일
#   1. iw 설치 (apt) — 도우미가 옛 가상 인터페이스(broonap0)를 지울 때 쓴다.
#   2. 도우미 broon-ap → /usr/local/sbin/broon-ap (root 소유 0755). broon-ap 를 고치면 이 스크립트를 다시 돌린다.
#   3. /etc/sudoers.d/broon-ap : 이 스크립트를 sudo 로 돌린 사용자에게만 '비밀번호 없이 broon-ap' 를 허락 (visudo 로 검사).
#   4. ufw 가 켜져 있으면 Wi-Fi 장치(핫스팟이 뜨는 곳)로 들어오는 DHCP(67/udp) · DNS(53) · 휴대폰 UDP(47318) 를 허락.
#      DNS · UDP 는 핫스팟 주소(10.42.0.1)로 오는 것만 — Wi-Fi 로 인터넷에 붙어 있을 때 바깥에서 오는 것은 그대로 막힌다.
#      처음 설계의 broonap0 규칙이 있으면 지운다.
# ★핫스팟을 켜지는 않는다★ — 인터넷은 그대로다. 켜는 것은 joy 노드다 (켜질 때 만들고 끝날 때 내린다 · hotspot:=false 면 안 만든다).
# 되돌리기 : sudo rm /usr/local/sbin/broon-ap /etc/sudoers.d/broon-ap  (ufw 규칙은 sudo ufw status numbered 로 보고 지운다)
set -eu
UDP_PORT=47318        # broon/wireless.py 의 UDP_PORT 와 같다
AP_IP=10.42.0.1       # broon/wireless.py 의 AP_IP 와 같다
OLD_IFACE=broonap0    # 처음 설계의 가상 인터페이스 (2026-11-23 폐기)

[ "$(id -u)" = 0 ] || { echo "sudo bash $0 로 돌릴 것"; exit 1; }
USER_NAME=${SUDO_USER:-}
[ -n "$USER_NAME" ] && [ "$USER_NAME" != root ] || { echo "sudo 로 돌려야 한다 (그 사용자에게 허락한다)"; exit 1; }
HERE=$(cd "$(dirname "$0")" && pwd)
[ -f "$HERE/broon-ap" ] || { echo "$HERE/broon-ap 이 없다"; exit 1; }

echo "[1/4] iw 설치"
command -v iw >/dev/null 2>&1 || apt-get install -y iw

echo "[2/4] 도우미 → /usr/local/sbin/broon-ap"
bash -n "$HERE/broon-ap"
install -o root -g root -m 0755 "$HERE/broon-ap" /usr/local/sbin/broon-ap

echo "[3/4] sudoers — $USER_NAME 가 비밀번호 없이 broon-ap 만"
tmp=$(mktemp)
echo "$USER_NAME ALL=(root) NOPASSWD: /usr/local/sbin/broon-ap" > "$tmp"
visudo -cf "$tmp" >/dev/null
install -o root -g root -m 0440 "$tmp" /etc/sudoers.d/broon-ap
rm -f "$tmp"

echo "[4/4] 방화벽"
if command -v ufw >/dev/null 2>&1 && ufw status | grep -q "Status: active"; then
    for spec in "to any port 67 proto udp" "to any port 53" "to any port $UDP_PORT proto udp"; do
        # shellcheck disable=SC2086
        ufw delete allow in on "$OLD_IFACE" $spec >/dev/null 2>&1 || true    # 처음 설계의 규칙 (없으면 그냥 지나간다)
    done
    WIFI=$(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | awk -F: -v old="$OLD_IFACE" '$2 == "wifi" && $1 != old {print $1; exit}')
    if [ -n "$WIFI" ]; then
        ufw allow in on "$WIFI" to any port 67 proto udp
        ufw allow in on "$WIFI" to "$AP_IP" port 53
        ufw allow in on "$WIFI" to "$AP_IP" port "$UDP_PORT" proto udp
        echo "  ufw 규칙 추가 (DHCP · DNS · UDP $UDP_PORT, $WIFI 에서만 — DNS · UDP 는 $AP_IP 로 오는 것만)"
    else
        echo "  ⚠️ Wi-Fi 장치를 못 찾았다 — ufw 규칙을 넣지 못했다"
    fi
else
    echo "  ufw 꺼짐 — 할 것 없음"
fi
echo "완료. 핫스팟은 joy 노드가 켜질 때 만들고 끝날 때 내린다 (hotspot:=false 면 안 만든다). 손으로 : sudo /usr/local/sbin/broon-ap up|down|status"
