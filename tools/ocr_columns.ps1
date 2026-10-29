# Split a screenshot into vertical strips, upscale each, OCR each strip.
# Built for numeric tables: one strip per column gives clean single-column text.
# Usage: powershell -File tools\ocr_columns.ps1 <image> "40-66,76-240,266-340,370-470" [scale]
param([Parameter(Mandatory=$true)][string]$Path,
      [Parameter(Mandatory=$true)][string]$Ranges,
      [double]$Scale = 3.0,
      [int]$YMin = 0,
      [int]$YMax = 100000)

[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Runtime.WindowsRuntime

$asTaskGeneric = ([System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object {
    $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and
    $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1'
})[0]
function Await($WinRtTask, $ResultType) {
    $asTask = $asTaskGeneric.MakeGenericMethod($ResultType)
    $netTask = $asTask.Invoke($null, @($WinRtTask))
    $netTask.Wait(-1) | Out-Null
    $netTask.Result
}
[Windows.Storage.StorageFile, Windows.Foundation, ContentType=WindowsRuntime] | Out-Null
[Windows.Graphics.Imaging.BitmapDecoder, Windows.Foundation, ContentType=WindowsRuntime] | Out-Null
[Windows.Media.Ocr.OcrEngine, Windows.Foundation, ContentType=WindowsRuntime] | Out-Null

$engine = [Windows.Media.Ocr.OcrEngine]::TryCreateFromUserProfileLanguages()
if (-not $engine) { Write-Error 'OCR engine unavailable'; exit 1 }

function Ocr-File([string]$p) {
    $f  = Await ([Windows.Storage.StorageFile]::GetFileFromPathAsync($p)) ([Windows.Storage.StorageFile])
    $s  = Await ($f.OpenAsync([Windows.Storage.FileAccessMode]::Read)) ([Windows.Storage.Streams.IRandomAccessStream])
    $d  = Await ([Windows.Graphics.Imaging.BitmapDecoder]::CreateAsync($s)) ([Windows.Graphics.Imaging.BitmapDecoder])
    $bm = Await ($d.GetSoftwareBitmapAsync()) ([Windows.Graphics.Imaging.SoftwareBitmap])
    $r  = Await ($engine.RecognizeAsync($bm)) ([Windows.Media.Ocr.OcrResult])
    foreach ($line in $r.Lines) { $line.Text }
}

$src = [System.Drawing.Image]::FromFile($Path)
$tmpDir = Join-Path $env:TEMP 'ocrcols'
$null = New-Item -ItemType Directory -Force -Path $tmpDir

$y0 = [Math]::Max(0, $YMin)
$y1 = [Math]::Min($src.Height, $YMax)
$bandH = $y1 - $y0
if ($bandH -le 1) { Write-Error "empty y band $y0..$y1"; exit 1 }

foreach ($rng in $Ranges.Split(',')) {
    $parts = $rng.Split('-')
    $x0 = [int]$parts[0]
    $x1 = [int]$parts[1]
    if ($x1 -gt $src.Width) { $x1 = $src.Width }
    $w = $x1 - $x0
    if ($w -le 1) { continue }

    $rect = New-Object System.Drawing.Rectangle($x0, $y0, $w, $bandH)
    $crop = New-Object System.Drawing.Bitmap($w, $bandH)
    $g = [System.Drawing.Graphics]::FromImage($crop)
    $g.DrawImage($src, (New-Object System.Drawing.Rectangle(0, 0, $w, $bandH)), $rect,
                 [System.Drawing.GraphicsUnit]::Pixel)
    $g.Dispose()

    $big = New-Object System.Drawing.Bitmap([int]($w * $Scale), [int]($bandH * $Scale))
    $g2 = [System.Drawing.Graphics]::FromImage($big)
    $g2.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
    $g2.DrawImage($crop, 0, 0, $big.Width, $big.Height)
    $g2.Dispose()

    $out = Join-Path $tmpDir ("col_{0}_{1}.png" -f $x0, $x1)
    $big.Save($out, [System.Drawing.Imaging.ImageFormat]::Png)
    $crop.Dispose(); $big.Dispose()

    "=== column x $x0..$x1 ==="
    Ocr-File $out
}
$src.Dispose()
