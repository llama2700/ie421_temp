#!/bin/bash

IFACES=()
if [[ "$(uname)" == "Darwin" ]]; then
    while IFS= read -r iface; do
        IFACES+=("$iface")
    done < <(networksetup -listallhardwareports 2>/dev/null | awk '/^Device:/ {print $2}' | head -5)
    get_ip() { ipconfig getifaddr "$1" 2>/dev/null || true; }
else
    while IFS= read -r iface; do
        [[ "$iface" == "lo" ]] && continue
        IFACES+=("$iface")
    done < <(ls /sys/class/net/ | head -6)
    IFACES=("${IFACES[@]:0:5}")
    get_ip() {
        ip -4 addr show "$1" 2>/dev/null | awk '/inet / {split($2,a,"/"); print a[1]; exit}'
    }
fi

if [[ ${#IFACES[@]} -eq 0 ]]; then
    echo "No network interfaces found."
    exit 1
fi

# Display interfaces
get_nic_name() {
    if [[ "$(uname)" == "Darwin" ]]; then
        networksetup -listallhardwareports 2>/dev/null | awk -v dev="$1" '
            /^Hardware Port:/ { name=$0; sub(/^Hardware Port: /,"",name) }
            /^Device:/ && $2 == dev { print name; exit }
        '
    else
        if [[ -f "/sys/class/net/$1/device/label" ]]; then
            cat "/sys/class/net/$1/device/label" 2>/dev/null
        else
            ethtool -i "$1" 2>/dev/null | awk -F: '/^driver/ {gsub(/^ /,"",$2); print $2}'
        fi
    fi
}

echo "Available interfaces:"
for i in "${!IFACES[@]}"; do
    iface="${IFACES[$i]}"
    ip=$(get_ip "$iface")
    nic_name=$(get_nic_name "$iface")
    info=""
    [[ -n "$nic_name" ]] && info="$nic_name"
    [[ -n "$ip" ]] && info="${info:+$info, }$ip"
    if [[ -n "$info" ]]; then
        echo "  [$i] $iface ($info)"
    else
        echo "  [$i] $iface"
    fi
done

# Prompt user
read -rp $'\nSelect interface [0-'"$((${#IFACES[@]}-1))"']: ' choice

if ! [[ "$choice" =~ ^[0-9]+$ ]] || (( choice < 0 || choice >= ${#IFACES[@]} )); then
    echo "Invalid selection."
    exit 1
fi

IFACE="${IFACES[$choice]}"
IP=$(get_ip "$IFACE")
if [[ -z "$IP" ]]; then
    echo "No IP address on $IFACE, using 0.0.0.0"
    IP="0.0.0.0"
fi

# \xec\xeb\xca\xfe - 4B * 256
PATTERN=$(printf '\\xec\\xeb\\xca\\xfe%.0s' {1..256})
PAYLOAD_FILE=$(mktemp)
printf "$PATTERN" > "$PAYLOAD_FILE"
trap 'rm -f "$PAYLOAD_FILE"' EXIT

echo ""
echo "Broadcasting on $IFACE ($IP) -> 255.255.255.255:9999"
echo "Payload: 1024 bytes of 0xECEBCAFE pattern"
echo "Press 'x' to stop."
echo ""

COUNT=0

cleanup() {
    echo ""
    echo "Stopped. Total packets sent: $COUNT"
    # Restore terminal
    stty sane 2>/dev/null
    exit 0
}

trap cleanup INT TERM

# Set terminal to raw mode for keypress detection
stty -icanon -echo min 0 time 0 2>/dev/null

while true; do
    python3 -c "
import socket, struct
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind(('$IP', 0))
p = struct.pack('>I', 0xECEBCAFE) * 256
s.sendto(p, ('255.255.255.255', 9999))
s.close()
"
    COUNT=$((COUNT + 1))
    printf "\rPackets sent: %d" "$COUNT"

    # Check for keypress (non-blocking)
    key=$(dd bs=1 count=1 2>/dev/null) || true
    if [[ "$key" == "x" || "$key" == "X" ]]; then
        cleanup
    fi
done
