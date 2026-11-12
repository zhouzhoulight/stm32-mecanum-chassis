"""Offline F407VE25MHz nav build. No probe/serial access, never flashes."""
import hashlib,json,os,re,struct,subprocess,sys,shutil
from pathlib import Path
r=Path(__file__).resolve().parents[1];out=r/'build/nav1k';out.mkdir(parents=True,exist_ok=True)
subprocess.run([sys.executable,str(r/'scripts/optimize_generated_math.py'),'--root',str(r),'--report',str(out/'math_profile.json')],check=True)
gcc_path=os.environ.get('CHASSIS_GCC') or shutil.which('arm-none-eabi-gcc')
if not gcc_path:raise SystemExit('Set CHASSIS_GCC or add arm-none-eabi-gcc to PATH.')
gcc=Path(gcc_path)
flags=['-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-O2','-g3','-std=c11','-DSTM32F407xx','-DUSE_HAL_DRIVER','-DHSE_VALUE=25000000U','-ffunction-sections','-fdata-sections','-fstack-usage','-Wall','-Wextra','-Werror']
for p in ['include','src/nav','vendor/Drivers/STM32F4xx_HAL_Driver/Inc','vendor/Drivers/CMSIS/Include','vendor/Drivers/CMSIS/Device/ST/STM32F4xx/Include','arm/project/Core/Inc']:flags+=['-I'+str(r/p)]
sources=['src/nav_main.c','src/nav_startup.s','src/system_stm32f4xx.c','src/wit_imu.c','src/imu_service.c']
sources += [p.relative_to(r).as_posix() for p in sorted((r/'src/nav').glob('*.c'))]
sources += [p.relative_to(r).as_posix() for p in sorted((r/'src/nav/generated').glob('*.c'))]
duplicates={'rt_nonfinite.c','rtGetInf.c','rtGetNaN.c'}
sources += [p.relative_to(r).as_posix() for p in sorted((r/'src/nav/qp').glob('*.c')) if p.name not in duplicates]
sources += [p for p in re.findall(r'- file: ([^\s]+\.c)\b',(r/'chassis.cproject.yml').read_text(encoding='utf-8')) if p.startswith('vendor/')]
objects=[]
for s in sources:
 obj=out/(s.replace('/','_')+'.o');extra=['-Wno-unused-parameter'] if s.startswith('vendor/') else (['-Wno-misleading-indentation','-Wno-unused-function'] if s.endswith('nav_arm.c') else [])
 subprocess.run([str(gcc),*flags,*extra,'-c',str(r/s),'-o',str(obj)],check=True);objects.append(str(obj))
elf=out/'chassis-nav1k.elf'
subprocess.run([str(gcc),*flags,'-nostartfiles','--specs=nano.specs','--specs=nosys.specs','-T',str(r/'STM32F407VE_nav.ld'),'-Wl,--gc-sections','-Wl,-Map='+str(out/'nav1.map'),'-Wl,--print-memory-usage',*objects,'-lm','-o',str(elf)],check=True)
for fmt,ext in [('binary','bin'),('ihex','hex')]:subprocess.run([str(gcc.with_name('arm-none-eabi-objcopy.exe')),'-O',fmt,str(elf),str(elf.with_suffix('.'+ext))],check=True)
subprocess.run([str(gcc.with_name('arm-none-eabi-size.exe')),str(elf)],check=True)
b=elf.with_suffix('.bin').read_bytes();sp,pc=struct.unpack_from('<II',b)
assert sp==0x20020000 and pc&1 and 0x08000000<pc<0x08000000+len(b) and len(b)<384*1024
disasm=subprocess.check_output([str(gcc.with_name('arm-none-eabi-objdump.exe')),'-d',str(elf)],text=True)
(out/'nav1.disasm').write_text(disasm,encoding='utf-8')
metadata={'version':re.search(r'#define NAV_VERSION "([^"]+)"',(r/'include/nav/nav_types.h').read_text()).group(1),'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest(),'hse_mhz':25,'sysclk_mhz':168,'float_abi':'hard; entire image','flash_code_kib':384,'reserved_flash_sector':7,'stack_reserved':8192,'initial_sp':hex(sp),'reset_pc':hex(pc),'sources':sources,'validation':'build only; NOT flashed or vehicle-tested'}
dependencies={r/s for s in sources}|set((r/'include').rglob('*.h'))|set((r/'src/nav').rglob('*.h'))|set((r/'vendor').rglob('*.h'))|{r/'STM32F407VE_nav.ld'}
dependencies|={r/'arm/project/APP/arm_task.c',r/'arm/platform/profile_store.c'}|set((r/'arm/project/Core/Inc').rglob('*.h'))
metadata['source_sha256']={p.relative_to(r).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(dependencies)}
metadata['toolchain']=subprocess.check_output([str(gcc),'--version'],text=True).splitlines()[0]
(out/'build.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in metadata.items() if k not in {'sources','source_sha256'}},indent=2))
