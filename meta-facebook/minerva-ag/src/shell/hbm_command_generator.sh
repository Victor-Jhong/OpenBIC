#!/bin/bash

# HBM I2C Command Generator
# This script generates I2C commands to read HBM (High Bandwidth Memory) data
# when the athena read command is not available in the firmware.
#
# Usage: ./hbm_command_generator.sh <hbm 0-5> <chan 0-15>
# 
# The script outputs the exact I2C write and read commands needed to access
# HBM data using direct I2C commands as documented in hbm_command_guide.txt
#
# Hardware Configuration:
# - I2C Bus: I2C_5 (corresponds to I2C_BUS6 in code)
# - Write Address: 0x6A (7-bit address for setting offset)
# - Read Address: 0x6C (7-bit address for reading data)
# - Data Size: 8 bytes per channel

# HBM Memory Layout Configuration (from hbm_command_guide.txt)
HBM_BASE_OFFSET=0xD07C4000

# HBM Data Offsets
declare -a HBM_OFFSETS=(0x0EED 0x1327 0x1761 0x1B9B 0x1FD5 0x240F)
declare -a HBM_OFFSET_NAMES=("HBM0" "HBM1" "HBM2" "HBM3" "HBM4" "HBM5")

# I2C Hardware Configuration
I2C_BUS="I2C_5"           # corresponds to I2C_BUS6 in code
WRITE_ADDR="6A"           # 7-bit address for setting offset
READ_ADDR="6C"            # 7-bit address for reading data
CHANNEL_DATA_SIZE=8       # 8 bytes per channel

# Channel Data Structure (8 bytes per channel)
declare -a DATA_FIELDS=("max_temp_current" "temp_current" "min_temp_history" "max_temp_history" "sid0" "sid1" "sid2" "sid3")
declare -a DATA_DESCRIPTIONS=(
    "Current max temp"
    "Current temperature"
    "Historical min temp"
    "Historical max temp"
    "Sensor ID 0"
    "Sensor ID 1"
    "Sensor ID 2"
    "Sensor ID 3"
)

# Function to print usage
print_usage() {
    echo "HBM I2C Command Generator"
    echo "========================="
    echo "This script generates I2C commands to read HBM data when the athena read"
    echo "command is not available in the firmware."
    echo ""
    echo "Usage: $0 <hbm 0-5> <chan 0-15>"
    echo ""
    echo "Parameters:"
    echo "  hbm      : HBM number (0-5)"
    echo "  chan     : Channel number (0-15)"
    echo ""
    echo "Address Calculation Formula:"
    echo "  Full_Address = HBM_BASE_OFFSET + HBM_OFFSET[H] + (C * 8)"
    echo "               = 0xD07C4000 + HBM_OFFSET[H] + (C * 8)"
    echo ""
    echo "HBM Data Offsets:"
    for i in "${!HBM_OFFSETS[@]}"; do
        printf "  HBM %d: 0x%04X\n" $i ${HBM_OFFSETS[$i]}
    done
    echo ""
    echo "Examples:"
    echo "  $0 0 0     # Read HBM 0, Channel 0"
    echo "  $0 2 10    # Read HBM 2, Channel 10"
    echo "  $0 5 15    # Read HBM 5, Channel 15"
}

# Function to validate inputs
validate_inputs() {
    local hbm=$1
    local channel=$2
    
    if [[ ! $hbm =~ ^[0-5]$ ]]; then
        echo "Error: HBM must be between 0-5" >&2
        return 1
    fi
    
    if [[ ! $channel =~ ^([0-9]|1[0-5])$ ]]; then
        echo "Error: Channel must be between 0-15" >&2
        return 1
    fi
    
    return 0
}

# Function to calculate address (based on hbm_command_guide.txt formula)
calculate_address() {
    local hbm=$1
    local channel=$2
    local hbm_offset=${HBM_OFFSETS[$hbm]}
    
    # Address Calculation Formula:
    # Full_Address = HBM_BASE_OFFSET + HBM_OFFSET[H] + (C * 8)
    local full_address=$((HBM_BASE_OFFSET + hbm_offset + (channel * CHANNEL_DATA_SIZE)))
    echo $full_address
}

# Function to break address into bytes (Little Endian format)
break_address() {
    local address=$1
    local byte0=$((address & 0xFF))         # addr_byte0: LSB
    local byte1=$(((address >> 8) & 0xFF))  # addr_byte1
    local byte2=$(((address >> 16) & 0xFF)) # addr_byte2
    local byte3=$(((address >> 24) & 0xFF)) # addr_byte3: MSB
    
    printf "%02X %02X %02X %02X" $byte0 $byte1 $byte2 $byte3
}

# Function to generate I2C commands (following hbm_command_guide.txt format)
generate_commands() {
    local hbm=$1
    local channel=$2
    
    # Calculate address using the documented formula
    local address=$(calculate_address $hbm $channel)
    local addr_bytes=$(break_address $address)
    local hbm_offset_hex=$(printf "0x%04X" ${HBM_OFFSETS[$hbm]})
    
    echo "================================================================"
    echo "HBM I2C Command Generator - Reading HBM $hbm, Channel $channel"
    echo "================================================================"
    # echo ""
    # echo "Address Calculation:"
    # echo "  Base Address:    0x$(printf "%08X" $HBM_BASE_OFFSET)"
    # echo "  HBM $hbm Offset:    $hbm_offset_hex"
    # echo "  Channel Offset:  $channel * 8 = $((channel * 8))"
    # echo "  Full Address:    0x$(printf "%08X" $address)"
    # echo ""
    
    # Show address breakdown
    IFS=' ' read -ra BYTES <<< "$addr_bytes"
    # echo "Address Breakdown:"
    # echo "  addr_byte0 = 0x${BYTES[0]} (LSB)"
    # echo "  addr_byte1 = 0x${BYTES[1]}"
    # echo "  addr_byte2 = 0x${BYTES[2]}"
    # echo "  addr_byte3 = 0x${BYTES[3]} (MSB)"
    # echo ""
    
    echo "I2C Command Sequence:"
    echo "===================="
    echo ""
    
    # Step 1: Write offset address
    echo "Step 1: Write Offset Address"
    echo "----------------------------"
    # echo "Command format: i2c write <bus> <addr> D3 06 [addr_bytes] FF 10 FF"
    echo ""
    echo "i2c write $I2C_BUS $WRITE_ADDR D3 06 $addr_bytes FF 10 FF"
    echo ""
    # echo "Where:"
    # echo "  - D3 06: Fixed command prefix"
    # echo "  - $addr_bytes: Address bytes (LSB to MSB)"
    # echo "  - FF 10 FF: Fixed command suffix"
    # echo ""
    
    # Step 2: Read data
    echo "Step 2: Read Data"
    echo "-----------------"
    # echo "Command format: i2c read <bus> <addr> <reg> <bytes>"
    echo ""
    echo "i2c read $I2C_BUS $READ_ADDR 00 08"
    echo ""
    # echo "This reads 8 bytes of data from the previously set address."
    # echo ""
    
    # Data interpretation
    echo "Data Interpretation:"
    echo "==================="
    # echo ""
    # echo "After executing the read command, you will get 8 bytes of data:"
    echo ""
    echo "Response: XX XX XX XX XX XX XX XX"
    echo "          |  |  |  |  |  |  |  |"
    echo "          |  |  |  |  |  |  |  +-- Byte 7: sid3"
    echo "          |  |  |  |  |  |  +---- Byte 6: sid2"
    echo "          |  |  |  |  |  +------ Byte 5: sid1"
    echo "          |  |  |  |  +-------- Byte 4: sid0"
    echo "          |  |  |  +---------- Byte 3: max_temp_history"
    echo "          |  |  +------------ Byte 2: min_temp_history"
    echo "          |  +-------------- Byte 1: temp_current"
    echo "          +---------------- Byte 0: max_temp_current"
    echo ""
    
    # Show data field mapping
    # echo "Data Field Mapping:"
    # echo "| Byte | Data Field         | Description           |"
    # echo "|------|--------------------|----------------------|"
    # for i in "${!DATA_FIELDS[@]}"; do
    #     printf "| %-4d | %-18s | %-20s |\n" $i "${DATA_FIELDS[$i]}" "${DATA_DESCRIPTIONS[$i]}"
    # done
    # echo ""
}

# Main script
main() {
    # Check if exactly 2 arguments provided
    if [[ $# -ne 2 ]]; then
        echo "Error: Invalid number of arguments" >&2
        echo "" >&2
        print_usage
        exit 1
    fi
    
    local hbm=$1
    local channel=$2
    
    # Validate inputs
    if ! validate_inputs $hbm $channel; then
        echo "" >&2
        print_usage
        exit 1
    fi
    
    # Generate commands
    generate_commands $hbm $channel
}

# Run main function with all arguments
main "$@"