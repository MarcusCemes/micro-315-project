PROJECT = micro-315-project

# Path to the e-pick2_main-processor folder
GLOBAL_PATH = ../../lib/e-puck2_main-processor

CSRC += main.c audio.c comms.c lights.c sensors.c utils.c vendor/mpack.c

# Additional headers
INCDIR +=

# Disable optimisation for debugging, require a project clean
# Lower optimisation does seem to cause stack overflows more frequently
# USE_COPT += -O0

# Run the main Makefile
include $(GLOBAL_PATH)/Makefile
