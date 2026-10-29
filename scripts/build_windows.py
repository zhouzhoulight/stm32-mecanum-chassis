"""Offline Windows GCC build. No debugger or serial access."""
import hashlib,json,os,re,struct,subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'build/windows-gcc'
out.mkdir(parents=True,exist_ok=True)
gcc=Path(os.environ.get('CHASSIS_GCC',r'D:\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe'))
flags=['-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-Os','-g3','-std=c11',
       '-DSTM32F407xx','-DUSE_HAL_DRIVER','-DHSE_VALUE=25000000U',
       '-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror']
for p in ['include','vendor/Drivers/STM32F4xx_HAL_Driver/Inc','vendor/Drivers/CMSIS/Include',
          'vendor/Drivers/CMSIS/Device/ST/STM32F4xx/Include']:flags+=['-I'+str(ROOT/p)]
sources=re.findall(r'- file: ([^\s]+\.(?:c|s))\b',(ROOT/'chassis.cproject.yml').read_text(encoding='utf-8'))
assert sources and 'src/imu_service.c' in sources
objects=[]
for src in sources:
 obj=out/(src.replace('/','_')+'.o');args=[str(gcc),*flags]
 if src.startswith('vendor/'):args+=['-Wno-unused-parameter']
 subprocess.run([*args,'-c',str(ROOT/src),'-o',str(obj)],cwd=ROOT,check=True)
 objects.append(str(obj))
elf=out/'chassis-f407-test.elf'
subprocess.run([str(gcc),*flags,'-nostartfiles','--specs=nano.specs','--specs=nosys.specs',
               '-T',str(ROOT/'STM32F407VE.ld'),'-Wl,--gc-sections',
               '-Wl,-Map='+str(out/'chassis.map'),'-Wl,--print-memory-usage',
               *objects,'-o',str(elf)],cwd=ROOT,check=True)
for fmt,ext in [('binary','bin'),('ihex','hex')]:
 subprocess.run([str(gcc.with_name('arm-none-eabi-objcopy.exe')),'-O',fmt,str(elf),str(elf.with_suffix('.'+ext))],check=True)
subprocess.run([str(gcc.with_name('arm-none-eabi-size.exe')),str(elf)],check=True)
b=elf.with_suffix('.bin').read_bytes();sp,pc=struct.unpack_from('<II',b)
assert 0x20000000<sp<=0x20020000 and (pc&1) and 0x08000000<pc<0x08000000+len(b)
assert 8<len(b)<=512*1024
version=re.search(r'#define TEST_FIRMWARE_VERSION \"([^\"]+)\"',(ROOT/'include/motor_test.h').read_text(encoding='utf-8')).group(1)
meta={'version':version,'base':'v2.1.5, motor behavior unchanged',
      'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest(),'initial_sp':hex(sp),
      'reset_pc':hex(pc),'flash_limit_kb':512,'hse_mhz':25,'build_only':True}
(out/'build.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
print(json.dumps(meta,indent=2))
