#!/bin/bash

# WiFi Connection Script for WPA/WPA2 Enterprise PEAP
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration variables (will be prompted if not set)
SSID=""
USERNAME=""
PASSWORD=""

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_success() {
    echo -e "${CYAN}[SUCCESS]${NC} $1"
}

print_separator() {
    echo -e "${BLUE}──────────────────────────────────────────────────────────────${NC}"
}

# Check if running as root
check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This script must be run as root. Use sudo."
        exit 1
    fi
}

# Check for required tools
check_dependencies() {
    local missing=()
    
    if ! command -v nmcli &> /dev/null; then
        missing+=("network-manager")
    fi
    
    if ! command -v wpa_supplicant &> /dev/null; then
        missing+=("wpa_supplicant")
    fi
    
    if [[ ${#missing[@]} -gt 0 ]]; then
        log_error "Missing required packages: ${missing[*]}"
        log_info "Install with: sudo apt update && sudo apt install ${missing[*]}"
        exit 1
    fi
}

# Get user input
get_credentials() {
    print_separator
    log_info "Enter WiFi Connection Details"
    print_separator
    
    # Get SSID
    while [[ -z "$SSID" ]]; do
        read -p "Enter WiFi SSID: " SSID
        if [[ -z "$SSID" ]]; then
            log_error "SSID cannot be empty"
        fi
    done
    
    # Get username
    while [[ -z "$USERNAME" ]]; do
        read -p "Enter username: " USERNAME
        if [[ -z "$USERNAME" ]]; then
            log_error "Username cannot be empty"
        fi
    done
    
    # Get password (hidden input)
    while [[ -z "$PASSWORD" ]]; do
        read -s -p "Enter password: " PASSWORD
        echo
        if [[ -z "$PASSWORD" ]]; then
            log_error "Password cannot be empty"
        fi
    done
    
    # Confirm details
    echo
    log_info "Connection details:"
    echo -e "  SSID: ${BLUE}$SSID${NC}"
    echo -e "  Username: ${BLUE}$USERNAME${NC}"
    echo -e "  Password: ${BLUE}********${NC}"
    
    read -p "Are these details correct? (y/N): " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        log_error "Aborted by user"
        exit 1
    fi
}

# Check if already connected
check_current_connection() {
    local current_ssid=$(nmcli -t -f active,ssid dev wifi | grep '^yes:' | cut -d: -f2)
    if [[ "$current_ssid" == "$SSID" ]]; then
        log_info "Already connected to $SSID"
        return 0
    fi
    return 1
}

# Create wpa_supplicant configuration
create_wpa_config() {
    local config_file="/etc/wpa_supplicant/wpa_supplicant.conf"
    
    log_info "Creating WPA supplicant configuration..."
    
    # Backup existing config if it exists
    if [[ -f "$config_file" ]]; then
        cp "$config_file" "$config_file.backup.$(date +%Y%m%d_%H%M%S)"
        log_info "Backed up existing config to $config_file.backup"
    fi
    
    # Create new config
    cat > "$config_file" << EOL
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=US

network={
    ssid="$SSID"
    key_mgmt=WPA-EAP
    eap=PEAP
    identity="$USERNAME"
    password="$PASSWORD"
    phase2="auth=MSCHAPV2"
    priority=1
}
EOL
    
    chmod 600 "$config_file"
    log_success "WPA configuration created at $config_file"
}

# Create NetworkManager connection
create_nm_connection() {
    log_info "Creating NetworkManager connection..."
    
    # Delete existing connection if it exists
    if nmcli connection show | grep -q "$SSID"; then
        nmcli connection delete "$SSID" 2>/dev/null || true
        log_info "Removed existing connection for $SSID"
    fi
    
    # Create new connection
    nmcli connection add \
        type wifi \
        con-name "$SSID" \
        ifname wlan0 \
        ssid "$SSID" \
        -- \
        wifi-sec.key-mgmt wpa-eap \
        802-1x.eap peap \
        802-1x.phase2-auth mschapv2 \
        802-1x.identity "$USERNAME" \
        802-1x.password "$PASSWORD" \
        802-1x.anonymous-identity "" \
        802-1x.ca-cert ""
    
    log_success "NetworkManager connection created"
}

# Connect using NetworkManager
connect_nm() {
    log_info "Attempting to connect using NetworkManager..."
    
    if nmcli connection up "$SSID"; then
        log_success "Successfully connected to $SSID"
        return 0
    else
        log_error "Failed to connect using NetworkManager"
        return 1
    fi
}

# Test internet connection
test_connection() {
    log_info "Testing internet connection..."
    
    if ping -c 3 -W 5 8.8.8.8 &> /dev/null; then
        log_success "Internet connection working!"
        return 0
    else
        log_warning "No internet access detected"
        return 1
    fi
}

# Show connection status
show_status() {
    print_separator
    log_info "Connection Status:"
    print_separator
    
    echo -e "${BLUE}NetworkManager:${NC}"
    nmcli connection show --active
    
    echo -e "\n${BLUE}IP Address:${NC}"
    ip addr show wlan0 2>/dev/null | grep "inet " || echo "No IP address assigned"
    
    echo -e "\n${BLUE}Wireless Info:${NC}"
    iwconfig wlan0 2>/dev/null | grep -E "ESSID|Quality"
}

# Cleanup function
cleanup() {
    log_info "Cleaning up..."
    # Restore NetworkManager if we stopped it
    systemctl start NetworkManager 2>/dev/null || true
}

# Main function
main() {
    print_separator
    log_success "Jetson Orin Nano WiFi Setup for WPA/WPA2 Enterprise PEAP"
    print_separator
    
    check_root
    check_dependencies
    
    # Get credentials
    get_credentials
    
    # Check if already connected
    if check_current_connection; then
        show_status
        exit 0
    fi
    
    # Try NetworkManager method first
    log_info "=== NetworkManager ==="
    create_nm_connection
    
    if connect_nm; then
        test_connection
        show_status
        exit 0
    fi
    
    # If both methods fail
    log_error "All connection methods failed"
    cleanup
    exit 1
}

# Usage instructions
usage() {
    echo -e "${GREEN}Usage:${NC} sudo $0"
    echo ""
    echo -e "${CYAN}This script connects to WPA/WPA2 Enterprise PEAP networks${NC}"
    echo ""
    echo -e "${YELLOW}Features:${NC}"
    echo -e "  • Supports PEAP with MSCHAPv2"
    echo -e "  • No CA certificate required"
    echo -e "  • Tries NetworkManager"
    echo -e "  • Tests internet connection after setup"
    echo ""
    echo -e "${BLUE}Requirements:${NC}"
    echo -e "  • network-manager"
    echo -e "  • wpa_supplicant"
    echo ""
    echo -e "Run with: ${GREEN}sudo $0${NC}"
}

# Handle interrupts
trap cleanup EXIT INT TERM

# Check if help requested
if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
    usage
    exit 0
fi

# Run main function
main "$@"