uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
float32 wheel_rate_integ	# FW only and optional
float32[3] adaptive_k_term		# mc rate ctrl adaptive K term with constraint
float32[3] unlimit_adaptive_k_term      # mc rate ctrl adaptive K term without constraint

bool k_adaptive_on