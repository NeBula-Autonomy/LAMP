Header header
int32 streamscape_sequence_number
int32 source_node_id
int32 target_node_id
string source_node_label
string target_node_label
string source_robot
string target_robot
string[] comm_node_name_hops
int32[] comm_node_id_hops
SilvusStreamscapeNeighbor[] links

# The neighbor list doesn't include the noise or interference at the target node, so I include it here.
int32 target_noise_level_dBm
int32 target_interference_dB

int32 max_queue_size
int32 min_snr_dB

float32 min_data_rate_margin_Mbps

float32 max_total_data_rate_Mbps
float32 min_total_data_rate_Mbps

float32 max_loss_rate_percent

float32 min_estimated_udp_throughput_Mbps
float32 min_theoretical_udp_throughput_Mbps

int32 min_num_spatial_streams

int32 max_noise_level_dBm
int32 max_interference_dB



