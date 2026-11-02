$ErrorActionPreference='Stop'
$f4Uv4=$env:KEIL_UV4
if(-not $f4Uv4){$f4Cmd=Get-Command UV4.exe -ErrorAction SilentlyContinue;if($f4Cmd){$f4Uv4=$f4Cmd.Source}}
if(-not $f4Uv4 -or -not(Test-Path -LiteralPath $f4Uv4)){throw 'Set KEIL_UV4 or add UV4.exe to PATH.'}
$f4FromElf=$env:KEIL_FROMELF
if(-not $f4FromElf){$f4Cmd=Get-Command fromelf.exe -ErrorAction SilentlyContinue;if($f4Cmd){$f4FromElf=$f4Cmd.Source}}
if(-not $f4FromElf){$f4Candidate=Join-Path (Split-Path (Split-Path $f4Uv4 -Parent) -Parent) 'ARM\ARMCC\bin\fromelf.exe';if(Test-Path -LiteralPath $f4Candidate){$f4FromElf=$f4Candidate}}
if(-not $f4FromElf -or -not(Test-Path -LiteralPath $f4FromElf)){throw 'Set KEIL_FROMELF or add fromelf.exe to PATH.'}
Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'platform') -File | ForEach-Object {
    $f4Dest=if($_.Extension -eq '.h'){'project\Core\Inc'}else{'project\BSP'}
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $PSScriptRoot $f4Dest)
}
$f4Project=Join-Path $PSScriptRoot 'project\MDK-ARM\GC1_Arm_F407_R9D.uvprojx'
$f4Log=Join-Path $PSScriptRoot 'build.log'
$f4Build=Start-Process -FilePath $f4Uv4 -ArgumentList @('-b',$f4Project,'-t','ArmBenchF407','-j0','-o',$f4Log) -WindowStyle Hidden -Wait -PassThru
Get-Content -LiteralPath $f4Log
if($f4Build.ExitCode -gt 1 -or -not(Select-String -LiteralPath $f4Log -Pattern '0 Error\(s\)' -Quiet)){throw 'F407 R9D build failed'}
& $f4FromElf --bin --output (Join-Path $PSScriptRoot 'ArmBenchF407.bin') (Join-Path $PSScriptRoot 'project\MDK-ARM\F407Output\ArmBenchF407.axf')
if($LASTEXITCODE -ne 0){throw 'fromelf failed'}
Get-FileHash -LiteralPath (Join-Path $PSScriptRoot 'ArmBenchF407.bin')
