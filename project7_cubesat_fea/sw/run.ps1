# Compile the C# helpers against the SOLIDWORKS interop assemblies and run a step.
# Usage: powershell -NoProfile -ExecutionPolicy Bypass -File sw\run.ps1 Step0
param([string]$Step = 'Step0')
$ErrorActionPreference = 'Stop'
$here  = Split-Path -Parent $MyInvocation.MyCommand.Path
$redist = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist'
$refs = @("$redist\SolidWorks.Interop.sldworks.dll", "$redist\SolidWorks.Interop.cosworks.dll",
          "$redist\SolidWorks.Interop.swconst.dll")
foreach ($r in $refs) { [void][System.Reflection.Assembly]::LoadFrom($r) }
# C# needs all `using` lines before any type, so hoist them out of both files.
$lines = (Get-Content "$here\SwSim.cs") + (Get-Content "$here\$Step.cs")
$usings = $lines | Where-Object { $_ -match '^using [\w\.]+;' } | Sort-Object -Unique
$body   = $lines | Where-Object { $_ -notmatch '^using [\w\.]+;' }
Add-Type -TypeDefinition (($usings + $body) -join "`n") -ReferencedAssemblies $refs -Language CSharp
$out = Join-Path (Split-Path -Parent $here) 'results'
Invoke-Expression "[$Step]::Run('$out')"
