int32 my_node_id
string my_node_label
string my_robot_name
uint64 my_pg_key
int32 my_noise_level_dBm
int32 my_interference_dB
int32 my_txpw_actual_mW
float32 my_txpw_actual_dBm
int32 neighbor_node_id
string neighbor_node_label
uint64 neighbor_pg_key
string color
int32 queue_size
int32 snr_dB

# From nar: totalAirTime_percent = nar[0] / 10000; totalDataRate_Mbps = nar[1] / 1000000; lossRate_percent = nar[2]
float32 total_air_time_percent
float32 total_data_rate_Mbps
float32 loss_rate_percent

float32 theoretical_udp_throughput_Mbps
int32 num_spatial_streams

#from pcns
int32[] received_signal_power_dBm

#from API call
#int32[] received_signal_strength_indicator_dB # Same as received_signal_power_dBm above

int32 link_adaptation_modulation_coding_scheme
#from vgm
int32 guard_interval_mode
int32 cyclic_prefix_length
#string laMCS_description
