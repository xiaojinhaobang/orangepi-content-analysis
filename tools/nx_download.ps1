# Resume + parallel-chunk downloader for the NoMachine Windows installer.
param(
    [int]$Chunks = 3,
    [int]$MaxMinutes = 90,
    [int]$PollSeconds = 30
)

$ErrorActionPreference = 'Continue'
$curl  = 'C:\anaconda\Library\bin\curl.exe'
$url   = 'https://web9001.nomachine.com/download/10.1/Windows/nomachine-personal-edition_10.1.7_1_x64.exe'
$total = [int64]99985248
$main  = Join-Path $env:TEMP 'nomachine-personal-edition_10.1.7_1_x64.exe'
$dir   = Join-Path $env:TEMP 'nx_chunks'

Get-Process -Name curl -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 2

if (Test-Path $dir) { Remove-Item $dir -Recurse -Force }
New-Item -ItemType Directory -Path $dir -Force | Out-Null

$have = [int64]0
if (Test-Path $main) { $have = [int64](Get-Item $main).Length }
if ($have -ge $total) {
    Write-Output "already complete: $have bytes"
    exit 0
}

$remain = $total - $have
$step   = [int64][math]::Floor($remain / $Chunks)
Write-Output ("prefix {0:N2} MB, remaining {1:N2} MB in {2} chunks" -f ($have/1MB), ($remain/1MB), $Chunks)

$ranges = @()
$cursor = $have
for ($i = 1; $i -le $Chunks; $i++) {
    $start = $cursor
    if ($i -eq $Chunks) { $end = $total - 1 } else { $end = $start + $step - 1 }
    $ranges += ,@([int64]$start, [int64]$end)
    $cursor = $end + 1
}

for ($i = 0; $i -lt $ranges.Count; $i++) {
    $r  = $ranges[$i]
    $out = Join-Path $dir ("p{0}.bin" -f ($i + 1))
    $rangeArg = "-r " + $r[0] + "-" + $r[1]
    Write-Output ("chunk {0}: {1}" -f ($i + 1), $rangeArg)
    Start-Process -FilePath $curl -WindowStyle Hidden -ArgumentList @(
        '-s', '-L', '--retry', '500', '--retry-delay', '5', '--retry-all-errors',
        '--speed-time', '180', '--speed-limit', '100',
        '-o', $out, $rangeArg, $url
    ) | Out-Null
}

$deadline = (Get-Date).AddMinutes($MaxMinutes)
$last = [int64]0
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds $PollSeconds
    $sum = [int64]0
    Get-ChildItem $dir -Filter 'p*.bin' -ErrorAction SilentlyContinue | ForEach-Object { $sum += $_.Length }
    $done = $have + $sum
    $rate = ($sum - $last) / 1KB / $PollSeconds
    $last = $sum
    Write-Output ("{0}  {1:N2}/{2:N2} MB ({3:N1}%)  {4:N0} KB/s" -f (Get-Date -Format HH:mm:ss), ($done/1MB), ($total/1MB), ($done/$total*100), $rate)
    $allDone = $true
    foreach ($r in $ranges) {
        $need = $r[1] - $r[0] + 1
        $idx = [array]::IndexOf($ranges, $r)
        $f = Join-Path $dir ("p{0}.bin" -f ($idx + 1))
        if (-not (Test-Path $f) -or (Get-Item $f).Length -lt $need) { $allDone = $false }
    }
    if ($allDone) { break }
}

Get-Process -Name curl -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 2

$sum = [int64]0
Get-ChildItem $dir -Filter 'p*.bin' -ErrorAction SilentlyContinue | Sort-Object Name | ForEach-Object { $sum += $_.Length }
if (($have + $sum) -ne $total) {
    Write-Output ("INCOMPLETE: have {0} + chunks {1} = {2} of {3}" -f $have, $sum, ($have + $sum), $total)
    exit 2
}

$final = Join-Path $env:TEMP 'nomachine_10.1.7_x64_complete.exe'
$outFs = [System.IO.File]::Create($final)
$in = [System.IO.File]::OpenRead($main); $in.CopyTo($outFs); $in.Close()
Get-ChildItem $dir -Filter 'p*.bin' | Sort-Object Name | ForEach-Object {
    $s = [System.IO.File]::OpenRead($_.FullName); $s.CopyTo($outFs); $s.Close()
}
$outFs.Close()

Write-Output ("assembled: {0} ({1:N2} MB)" -f $final, ((Get-Item $final).Length/1MB))
$sig = Get-AuthenticodeSignature $final
Write-Output ("signature: {0}" -f $sig.Status)
Write-Output ("signer   : {0}" -f $sig.SignerCertificate.Subject)
