Import("env")
import os, glob

variant_base = os.path.join(env.subst("$PROJECT_PACKAGES_DIR"), "framework-arduinoststm32", "variants", "STM32G4xx")
dirs = glob.glob(os.path.join(variant_base, "*G474R*"))
if dirs:
    header = os.path.join(dirs[0], "variant_CUSTOM_G474RE.h")
    if not os.path.exists(header):
        with open(header, "w") as f:
            f.write('#include "variant_generic.h"\n')