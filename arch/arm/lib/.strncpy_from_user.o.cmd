cmd_arch/arm/lib/strncpy_from_user.o := arm-eabi-gcc -Wp,-MD,arch/arm/lib/.strncpy_from_user.o.d  -nostdinc -isystem /usr/bin/../lib/gcc/arm-eabi/4.5.0/include -Iinclude  -I/home/jacob/source/wildfire-kernel/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-msm/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=6 -march=armv6 -mtune=arm1136j-s -msoft-float -gdwarf-2     -c -o arch/arm/lib/strncpy_from_user.o arch/arm/lib/strncpy_from_user.S

deps_arch/arm/lib/strncpy_from_user.o := \
  arch/arm/lib/strncpy_from_user.S \
  include/linux/linkage.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  /home/jacob/source/wildfire-kernel/arch/arm/include/asm/linkage.h \
  /home/jacob/source/wildfire-kernel/arch/arm/include/asm/assembler.h \
    $(wildcard include/config/cpu/feroceon.h) \
  /home/jacob/source/wildfire-kernel/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/arm/thumb.h) \
    $(wildcard include/config/smp.h) \
  /home/jacob/source/wildfire-kernel/arch/arm/include/asm/hwcap.h \
  /home/jacob/source/wildfire-kernel/arch/arm/include/asm/errno.h \
  include/asm-generic/errno.h \
  include/asm-generic/errno-base.h \

arch/arm/lib/strncpy_from_user.o: $(deps_arch/arm/lib/strncpy_from_user.o)

$(deps_arch/arm/lib/strncpy_from_user.o):
