# Read text from a screenshot with the built-in Windows OCR engine.
# ASCII-only on purpose: Windows PowerShell 5.1 reads .ps1 as ANSI (GBK here),
# so non-ASCII source text can corrupt parsing.
# Usage: powershell -ExecutionPolicy Bypass -File tools\ocr_screenshot.ps1 <image> [-Scale 2] [-Rows]
param([Parameter(Mandatory=$true)][string]$Path,
      [double]$Scale = 2.0,
      [switch]$Rows,
      [switch]$Raw,
      [double]$YMin = 0,
      [double]$YMax = 1e9,
      [double]$XMin = -1,
      [double]$XMax = -1)

[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

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

$file    = Await ([Windows.Storage.StorageFile]::GetFileFromPathAsync($Path)) ([Windows.Storage.StorageFile])
$stream  = Await ($file.OpenAsync([Windows.Storage.FileAccessMode]::Read)) ([Windows.Storage.Streams.IRandomAccessStream])
$decoder = Await ([Windows.Graphics.Imaging.BitmapDecoder]::CreateAsync($stream)) ([Windows.Graphics.Imaging.BitmapDecoder])

# Upscale before OCR: small table text loses characters otherwise.
$xf = New-Object Windows.Graphics.Imaging.BitmapTransform

# Optional column crop, in original pixels: -XMin 270 -XMax 340 selects one column.
$srcW = $decoder.PixelWidth
$srcH = $decoder.PixelHeight
$x0 = 0.0
if ($XMin -ge 0 -and $XMax -gt $XMin) {
    $x0 = $XMin
    $srcW = $XMax - $XMin
}

$xf.Bounds = New-Object Windows.Graphics.Imaging.BitmapBounds
$xf.Bounds.X = [uint32][math]::Round($x0)
$xf.Bounds.Y = 0
$xf.Bounds.Width  = [uint32][math]::Round($srcW)
$xf.Bounds.Height = [uint32]$srcH
$xf.ScaledWidth  = [uint32]([math]::Round($srcW * $Scale))
$xf.ScaledHeight = [uint32]([math]::Round($srcH * $Scale))
$xf.InterpolationMode = [Windows.Graphics.Imaging.BitmapInterpolationMode]::Fant

$engine = [Windows.Media.Ocr.OcrEngine]::TryCreateFromUserProfileLanguages()
if (-not $engine) { Write-Error 'OCR engine unavailable (missing language pack)'; exit 1 }

$bitmap = Await ($decoder.GetSoftwareBitmapAsync(
    [Windows.Graphics.Imaging.BitmapPixelFormat]::Bgra8,
    [Windows.Graphics.Imaging.BitmapAlphaMode]::Premultiplied,
    $xf,
    [Windows.Graphics.Imaging.ExifOrientationMode]::IgnoreExifOrientation,
    [Windows.Graphics.Imaging.ColorManagementMode]::DoNotColorManage)) ([Windows.Graphics.Imaging.SoftwareBitmap])

$result = Await ($engine.RecognizeAsync($bitmap)) ([Windows.Media.Ocr.OcrResult])

# -- collect every word with its position ------------------------------------
$words = New-Object System.Collections.ArrayList
foreach ($line in $result.Lines) {
    foreach ($w in @($line.Words)) {
        [void]$words.Add([pscustomobject]@{
            y = [double]$w.BoundingRect.Y
            x = [double]$w.BoundingRect.X
            t = [string]$w.Text
        })
    }
}
Write-Warning ("words=" + $words.Count)

if ($Raw) {
    $words | Where-Object { $_.y -ge $YMin -and $_.y -le $YMax } |
        Sort-Object y, x |
        ForEach-Object { "{0,5} {1,5} {2}" -f [int]$_.y, [int]$_.x, $_.t }
    exit 0
}

if (-not $Rows) {
    foreach ($line in $result.Lines) { $line.Text }
    exit 0
}

# -- regroup words into table rows (same y) and columns (sorted x) -----------
# Rows are grouped by y tolerance; inside a row cells are ordered purely by x,
# because different columns of one visual row sit a few px apart vertically.
$sorted = $words | Sort-Object y, x
$tol = $Scale * 8
$row = New-Object System.Collections.ArrayList
$rowY = -99999.0

$flush = {
    if ($row.Count -gt 0) {
        $ordered = $row | Sort-Object x | ForEach-Object { $_.t }
        "y={0,5} | {1}" -f [int]$rowY, ($ordered -join ' | ')
    }
}

foreach ($w in $sorted) {
    if ([math]::Abs($w.y - $rowY) -gt $tol) {
        & $flush
        $rowY = $w.y
        $row = New-Object System.Collections.ArrayList
    }
    [void]$row.Add($w)
}
& $flush
