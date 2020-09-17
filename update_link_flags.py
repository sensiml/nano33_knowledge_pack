# Custom settings, as referred to as "extra_script" in platformio.ini
#
# See http://docs.platformio.org/en/latest/projectconf.html#extra-script

Import("env")

#env = DefaultEnvironment()
env.Dump()
env.Append(
    LINKFLAGS=[
        "-Os",
        "-Wl,--gc-sections,--relax",
        "-mthumb",
        "-mcpu=cortex-m4",
        "-mfloat-abi=softfp",
    ]
)

env.Append(
    CCFLAGS=[
        "-Os",  # optimize for size
        "-ffunction-sections",  # place each function in its own section
        "-fdata-sections",
        "-Wall",
        "-mthumb",
        "-mcpu=cortex-m4",
        "-mfloat-abi=softfp",
    ]
)

