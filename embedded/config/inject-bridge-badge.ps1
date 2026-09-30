<#
.SYNOPSIS
  Injects a GRBL bus lockout badge into the embedded ESP3D web page and
  regenerates the PROGMEM header, without a JavaScript toolchain.

.DESCRIPTION
  The embedded page ships as a gzipped PROGMEM blob that is normally
  produced by `npm run build` in embedded/ (webpack, then
  config/buildheader.js). This fork adds a small always-on indicator that
  shows when the offline remote owns the GRBL bus, but the WebUI source
  cannot be rebuilt here because Node is not installed.

  Instead this script patches the already built artifact, which needs no
  tooling beyond PowerShell:

    embedded/dist/index.html.gz          (pristine input, never modified)
      -> gunzip
      -> splice a <style> and a <script> before the final </body>
      -> gzip to embedded/dist/index.html.bridge.gz
      -> emit embedded/dist/embedded.h and esp3d/src/modules/http/embedded.h

  The byte to C array emitter is a faithful port of config/buildheader.js
  and reproduces the checked in header byte for byte, which is asserted
  before anything is written. If that self check ever fails the script
  aborts rather than overwriting firmware sources.

  The original blob is left untouched, so this is fully reversible: delete
  embedded/dist/index.html.bridge.gz and re-run with -Unpatch to restore.

  NOTE: any future real `npm run build` discards the badge, because webpack
  regenerates index.html.gz from embedded/src. Re-run this script afterwards
  to reapply it. The durable fix is to move the badge into
  embedded/src/index.js and rebuild with Node.

.PARAMETER Unpatch
  Regenerate the header from the pristine blob, removing the badge.
#>
[CmdletBinding()]
param([switch]$Unpatch)

$ErrorActionPreference = "Stop"

$root      = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$dist      = Join-Path $root "embedded\dist"
$srcTxt    = Join-Path $root "embedded\src"
$pristine  = Join-Path $dist "index.html.gz"
$patched   = Join-Path $dist "index.html.bridge.gz"
$distH     = Join-Path $dist "embedded.h"
$fwH       = Join-Path $root "esp3d\src\modules\http\embedded.h"

function Read-RawBytes([string]$path) {
    return ,([System.IO.File]::ReadAllBytes($path))
}

function Read-GzipBytes([string]$path) {
    $bytes = [System.IO.File]::ReadAllBytes($path)
    $mem = New-Object System.IO.MemoryStream(,$bytes)
    $gz = New-Object System.IO.Compression.GZipStream($mem, [System.IO.Compression.CompressionMode]::Decompress)
    $out = New-Object System.IO.MemoryStream
    $gz.CopyTo($out)
    $gz.Dispose(); $mem.Dispose()
    # the comma is required: PowerShell would otherwise unroll the byte array
    # into Object[] on return and every caller would see the wrong contents
    return ,$out.ToArray()
}

function Write-GzipBytes([byte[]]$bytes, [string]$path) {
    $mem = New-Object System.IO.MemoryStream
    $gz = New-Object System.IO.Compression.GZipStream($mem, [System.IO.Compression.CompressionMode]::Compress, $true)
    $gz.Write($bytes, 0, $bytes.Length)
    $gz.Dispose()
    [System.IO.File]::WriteAllBytes($path, $mem.ToArray())
}

# Faithful port of embedded/config/buildheader.js
function ConvertTo-EmbeddedHeader([byte[]]$bytes) {
    $sb = New-Object System.Text.StringBuilder
    [void]$sb.Append((Get-Content -Raw (Join-Path $srcTxt "header.txt")))
    [void]$sb.Append("#define tool_html_gz_size  " + $bytes.Length + "`n")
    [void]$sb.Append("const unsigned char tool_html_gz[" + $bytes.Length + "] PROGMEM = {`n   ")
    $nb = 0
    for ($i = 0; $i -lt $bytes.Length; $i++) {
        [void]$sb.Append(" 0x" + $bytes[$i].ToString("x2"))
        if ($i -lt $bytes.Length - 1) { [void]$sb.Append(",") }
        if ($nb -eq 15) { [void]$sb.Append("`n   "); $nb = 0 } else { $nb++ }
    }
    [void]$sb.Append("`n};`n")
    [void]$sb.Append((Get-Content -Raw (Join-Path $srcTxt "footer.txt")))
    $text = $sb.ToString() -replace "(?<!\r)\n", "`r`n"
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    return ,$utf8NoBom.GetBytes($text)
}

# ---- self check: emitter must reproduce the checked in header exactly ----
# The header embeds the *compressed* payload, so feed it the raw blob.
$selfCheck = ConvertTo-EmbeddedHeader (Read-RawBytes $pristine)
$onDisk = [System.IO.File]::ReadAllBytes($fwH)
if ($selfCheck.Length -ne $onDisk.Length) {
    throw ("self check FAILED: emitter produced {0} bytes, header on disk is {1}. " -f $selfCheck.Length, $onDisk.Length) + "Refusing to write."
}
for ($i = 0; $i -lt $onDisk.Length; $i++) {
    if ($selfCheck[$i] -ne $onDisk[$i]) {
        throw "self check FAILED: first difference at byte $i. Refusing to write."
    }
}
Write-Host "[ok] emitter reproduces the checked in header byte for byte"

# ---- build the page to embed ----
if ($Unpatch) {
    $gzOut = $pristine
    Write-Host "[ok] unpatched, using pristine blob"
} else {
    $html = [System.Text.Encoding]::UTF8.GetString((Read-GzipBytes $pristine))
    $marker = "</body>"
    $at = $html.LastIndexOf($marker)
    if ($at -lt 0) { throw "no </body> found in the embedded page" }

    $inject = @'
<style id="esp3dBridgeBadgeCss">
#esp3dBridgeBadge{position:fixed;right:10px;bottom:10px;z-index:2147483000;display:none;
padding:6px 12px;border-radius:6px;font:600 13px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
color:#fff;box-shadow:0 2px 8px rgba(0,0,0,.35);pointer-events:none;max-width:60vw}
</style>
<script id="esp3dBridgeBadgeJs">
(function(){var ID="esp3dBridgeBadge",el=null;
function ensure(){el=document.getElementById(ID);
if(!el){el=document.createElement("div");el.id=ID;document.body.appendChild(el);}return el;}
function paint(s){var e=ensure();
if(s==="remote"){e.textContent="TX LOCKED - the offline remote has the bus";e.style.background="#b3261e";e.style.display="block";}
else if(s==="alarm"){e.textContent="TX LOCKED - controller in ALARM";e.style.background="#8a5a00";e.style.display="block";}
else{e.style.display="none";}}
function poll(){fetch("bridge",{cache:"no-store"}).then(function(r){return r.text();})
.then(function(t){paint((t||"").trim());}).catch(function(){paint("");});}
ensure();poll();setInterval(poll,1000);})();
</script>
'@

    $html = $html.Substring(0, $at) + $inject + $html.Substring($at)
    Write-GzipBytes ([System.Text.Encoding]::UTF8.GetBytes($html)) $patched
    $gzOut = $patched
    Write-Host ("[ok] injected badge, page {0} -> {1} bytes, gzip {2} -> {3} bytes" -f `
        (Read-GzipBytes $pristine).Length, $html.Length, `
        (Get-Item $pristine).Length, (Get-Item $patched).Length)
}

# ---- regenerate headers ----
$header = ConvertTo-EmbeddedHeader (Read-RawBytes $gzOut)
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllBytes($distH, $header)
[System.IO.File]::WriteAllBytes($fwH, $header)
Write-Host "[ok] wrote embedded\dist\embedded.h and esp3d\src\modules\http\embedded.h"
