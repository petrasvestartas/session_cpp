$ErrorActionPreference = 'Stop'
$results = Join-Path $PWD 'diagnostic-results'
New-Item -ItemType Directory -Path $results | Out-Null
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$vs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw 'No Visual Studio C++ installation found' }
& (Join-Path $vs 'Common7/Tools/Launch-VsDevShell.ps1') -Arch amd64 -HostArch amd64 -SkipAutomaticLocation
Get-Command cl | Format-List * | Out-File (Join-Path $results 'compiler-command.txt')
Push-Location diagnostics/rtree
try {
  $configs = @(
    @{ Name = 'O2'; Flags = @('/O2', '/Ob2') },
    @{ Name = 'Od'; Flags = @('/Od') },
    @{ Name = 'O2-strict'; Flags = @('/O2', '/Ob2', '/fp:strict') },
    @{ Name = 'O2-noinline'; Flags = @('/O2', '/Ob0') },
    @{ Name = 'O2-novec'; Flags = @('/O2', '/Ob2', '/Qvec-') }
  )
  $receipts = @()
  foreach ($config in $configs) {
    $out = Join-Path $results $config.Name
    New-Item -ItemType Directory -Path $out | Out-Null
    $common = @('/nologo', '/EHsc', '/MD', '/DNDEBUG', '/D_MBCS', '/DWIN32', '/D_WINDOWS', '/D_CRT_SECURE_NO_WARNINGS', '/std:c++latest', '/W4', '/utf-8', '/Bv')
    foreach ($program in @('exact_repro', 'reducer_repro', 'ancestor_probe')) {
      $source = if ($program -eq 'ancestor_probe') { 'probe/exact_repro.cpp' } else { "${program}.cpp" }
      $exe = Join-Path $out ($program + '.exe')
      $obj = Join-Path $out ($program + '.obj')
      $asm = Join-Path $out ($program + '.asm')
      $arguments = $common + $config.Flags + @($source, "/Fe:$exe", "/Fo:$obj", '/FAs', "/Fa$asm")
      $arguments | ConvertTo-Json | Set-Content (Join-Path $out ($program + '-compile-arguments.json'))
      & cl @arguments 2>&1 | Tee-Object -FilePath (Join-Path $out ($program + '-compile.log'))
      if ($LASTEXITCODE -ne 0) { throw "Compile failed: $program $($config.Name)" }
      Get-FileHash -Algorithm SHA256 $exe | ConvertTo-Json | Set-Content (Join-Path $out ($program + '-exe-sha256.json'))
    }
    $exact = Join-Path $out 'exact_repro.exe'
    foreach ($mode in @('unchanged', '0', '1')) {
      if ($mode -eq 'unchanged') { & $exact > (Join-Path $out 'exact-unchanged.log') 2>&1 }
      else { & $exact $mode > (Join-Path $out ('exact-' + $mode + '.log')) 2>&1 }
      $code = $LASTEXITCODE
      $receipts += @{ config = $config.Name; program = 'exact'; mode = $mode; exit_code = $code }
      if ($code -notin @(0, 1)) { throw "Unexpected diagnostic exit $code" }
    }
    $probe = Join-Path $out 'ancestor_probe.exe'
    foreach ($mode in @('unchanged', '0', '1')) {
      if ($mode -eq 'unchanged') { & $probe > (Join-Path $out 'probe-unchanged.log') 2>&1 }
      else { & $probe $mode > (Join-Path $out ('probe-' + $mode + '.log')) 2>&1 }
      $code = $LASTEXITCODE
      $receipts += @{ config = $config.Name; program = 'ancestor-probe'; mode = $mode; exit_code = $code }
      if ($code -notin @(0, 1)) { throw "Unexpected ancestor probe exit $code" }
    }
    $reducer = Join-Path $out 'reducer_repro.exe'
    & $reducer > (Join-Path $out 'reducer-full.json') 2> (Join-Path $out 'reducer-full.stderr')
    $code = $LASTEXITCODE
    $receipts += @{ config = $config.Name; program = 'reducer-full'; exit_code = $code }
    if ($code -notin @(0, 1)) { throw "Unexpected reducer diagnostic exit $code" }
    if ($config.Name -eq 'O2' -and $code -eq 1) {
      foreach ($target in @(394, 395)) {
        python minimize.py --exe $reducer --target $target --output (Join-Path $out ('reduce-' + $target))
        if ($LASTEXITCODE -ne 0) { throw 'Reducer process failed' }
      }
    }
  }
  # Candidate variants preserve the controls above and use normal optimized flags.
  foreach ($config in ($configs | Where-Object { $_.Name -in @('O2', 'O2-strict') })) {
    $out = Join-Path $results ('candidate-' + $config.Name)
    New-Item -ItemType Directory -Path $out | Out-Null
    foreach ($program in @('exact_repro', 'reducer_repro', 'ancestor_probe')) {
      $source = if ($program -eq 'ancestor_probe') { 'candidate/probe/exact_repro.cpp' } else { "candidate/${program}.cpp" }
      $exe = Join-Path $out ($program + '.exe')
      $obj = Join-Path $out ($program + '.obj')
      $asm = Join-Path $out ($program + '.asm')
      $arguments = $common + $config.Flags + @($source, "/Fe:$exe", "/Fo:$obj", '/FAs', "/Fa$asm")
      $arguments | ConvertTo-Json | Set-Content (Join-Path $out ($program + '-compile-arguments.json'))
      & cl @arguments 2>&1 | Tee-Object -FilePath (Join-Path $out ($program + '-compile.log'))
      if ($LASTEXITCODE -ne 0) { throw "Candidate compile failed: $program $($config.Name)" }
      Get-FileHash -Algorithm SHA256 $exe | ConvertTo-Json | Set-Content (Join-Path $out ($program + '-exe-sha256.json'))
    }
    foreach ($program in @('exact_repro', 'ancestor_probe')) {
      $exe = Join-Path $out ($program + '.exe')
      foreach ($mode in @('unchanged', '0', '1')) {
        $log = Join-Path $out ($program + '-' + $mode + '.log')
        if ($mode -eq 'unchanged') { & $exe > $log 2>&1 }
        else { & $exe $mode > $log 2>&1 }
        $code = $LASTEXITCODE
        $receipts += @{ config = ('candidate-' + $config.Name); program = $program; mode = $mode; exit_code = $code }
        if ($code -notin @(0, 1)) { throw "Unexpected candidate probe exit $code" }
      }
    }
    $exe = Join-Path $out 'reducer_repro.exe'
    & $exe > (Join-Path $out 'reducer-full.json') 2> (Join-Path $out 'reducer-full.stderr')
    $code = $LASTEXITCODE
    $receipts += @{ config = ('candidate-' + $config.Name); program = 'reducer-full'; exit_code = $code }
    if ($code -notin @(0, 1)) { throw "Unexpected candidate reducer exit $code" }
    & $exe --target 394 --indices candidate/ten-indices.txt > (Join-Path $out 'reducer-ten.json') 2> (Join-Path $out 'reducer-ten.stderr')
    $code = $LASTEXITCODE
    $receipts += @{ config = ('candidate-' + $config.Name); program = 'reducer-ten'; exit_code = $code }
    if ($code -notin @(0, 1)) { throw "Unexpected candidate ten-box exit $code" }
  }
  $receipts | ConvertTo-Json -Depth 8 | Set-Content (Join-Path $results 'exit-receipts.json')
  Get-ChildItem -Recurse -File | Get-FileHash -Algorithm SHA256 | ConvertTo-Json | Set-Content (Join-Path $results 'source-sha256.json')
} finally { Pop-Location }

# A complete diagnostic experiment can contain failing probes; receipts carry each result.
exit 0
