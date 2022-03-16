int32 node_id
string node_label
string robot_name
uint64 pg_key
int32 last_updated_s
# bool isRefreshed

string gateway_ip
string ipaddr
string netmask

SilvusStreamscapeNeighbor[] neighbors

int32 frag_thresh_bytes
int32 beam_form_enable
string board_type
float32 burst_time_ms
int32 bandwidth_MHz
int32 contention_window_minimum
int32 full_power
int32 freq_MHz

int32 gps_status
float32 latitude
float32 longitude

int32 max_link_distance_m
int32 max_speed_mph

string mcast0_config
string mcast1_config
string mcast2_config
string mcast3_config
string mcast4_config

int32 link_adaptation_mode
int32 link_adaptation_disable
int32 modulation_coding_scheme
#from vgm
int32 guard_interval_mode
int32 cyclic_prefix_length

int32 noise_level_dBm
int32 interference_dB
int32 number_retries
int32 rts_retries
int32 queue_size
int32 routing_beacon_mcs
int32 routing_beacon_period_ms
key_value_int32_int32[] next_hop

int32 txpw_requested_mW
int32 txpw_actual_mW

float32 txpw_requested_dBm
float32 txpw_actual_dBm

# From nar: totalAirTime_percent = nar[0] / 10000; totalDataRate_Mbps = nar[1] / 1000000; lossRate_percent = nar[2]
float32 total_air_time_percent
float32 total_data_rate_Mbps
float32 loss_rate_percent

# from trs
float32 input_unicast_rate_Mbps
float32 input_multicast_rate_Mbps
float32 input_dropped_rate_Mbps
float32 forward_unicast_rate_Mbps
float32 forward_multicast_rate_Mbps