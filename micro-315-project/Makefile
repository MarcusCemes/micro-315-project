PROJECT = micro-315-project

# Path to the e-pick2_main-processor folder
GLOBAL_PATH = ../../lib/e-puck2_main-processor

CSRC += main.c 			\
		audio.c 		\
		comms.c 		\
		lights.c 		\
		localisation.c  \
		sensors.c 		\
		speaker.c 		\
		utils.c 		\
		vendor/mpack.c

# Additional headers
INCDIR +=

# Disable optimisation for debugging, requires a project clean
# USE_COPT += -O0

# Using -O0 globally increases memory usage signficiantly, causing
# stack overflows. Instead, consider opting out of optimisation
# for specific files/functions:
#
#   #pragma GCC push_options
#   #pragma GCC optimize ("O0")
#   ...
#   #pragma GCC pop_options

# Run the main Makefile
include $(GLOBAL_PATH)/Makefile
