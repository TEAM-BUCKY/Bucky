Import("env")
import os

# The STM32 core includes variant_<BOARD_ID>.h; custom boards have none, so
# create a shim in the board's variant dir that forwards to variant_generic.h.
board = env.BoardConfig()
variant_dir = os.path.join(env.subst("$PROJECT_PACKAGES_DIR"), "framework-arduinoststm32", "variants", board.get("build.variant"))
if os.path.isdir(variant_dir):
    header = os.path.join(variant_dir, "variant_%s.h" % env.subst("$BOARD").upper())
    if not os.path.exists(header):
        with open(header, "w") as f:
            f.write('#include "variant_generic.h"\n')
