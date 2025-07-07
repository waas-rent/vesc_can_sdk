# GDB configuration for VESC CAN SDK debugging

# Set pretty printing
set print pretty on
set print array on
set print array-indexes on

# Set source path
directory .

# Useful breakpoints for VESC CAN SDK
# Uncomment the ones you want to use:

# Break on buffer allocation
break vesc_process_can_frame_internal if packet_type == 0x01  # CAN_PACKET_FILL_RX_BUFFER

# Break on buffer processing
# break vesc_process_can_frame_internal if packet_type == 0x03  # CAN_PACKET_PROCESS_RX_BUFFER

# Break on specific controller ID
# break vesc_process_can_frame_internal if (id & 0xFF) == 1

# Break on debug output
# break vesc_debug_output

# Break on command sending
# break vesc_send_command

# Show useful info when stopping
define vesc_info
    printf "=== VESC CAN SDK Debug Info ===\n"
    printf "Controller ID: %d\n", controller_id
    printf "Packet Type: 0x%02X\n", packet_type
    printf "Data Length: %d\n", len
    if (buf_idx >= 0)
        printf "Buffer Index: %d\n", buf_idx
        printf "Buffer Active: %s\n", sdk_state.rx_buffers[buf_idx].active ? "true" : "false"
    end
    printf "==============================\n"
end

# Show buffer state
define show_buffers
    printf "=== VESC RX Buffers ===\n"
    for (int i = 0; i < 4; i++)
        printf "Buffer[%d]: active=%s, controller_id=%d, offset=%d\n", 
               i, 
               sdk_state.rx_buffers[i].active ? "true" : "false",
               sdk_state.rx_buffers[i].buffer[0],
               sdk_state.rx_buffers[i].offset
    end
    printf "=======================\n"
end

# Show CAN frame info
define show_can_frame
    printf "=== CAN Frame ===\n"
    printf "ID: 0x%03X\n", id
    printf "Controller ID: %d\n", controller_id
    printf "Packet Type: 0x%02X (%s)\n", packet_type, vesc_debug_get_packet_name(packet_type)
    printf "Length: %d\n", len
    printf "Data: "
    for (int i = 0; i < len && i < 8; i++)
        printf "%02X ", data[i]
    printf "\n"
    printf "===============\n"
end 