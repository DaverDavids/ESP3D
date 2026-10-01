<#
.SYNOPSIS
  Injects a GRBL bus lockout badge into the embedded ESP3D web page and
  regenerates the PROGMEM header, without a JavaScript toolchain.

.DESCRIPTION
  The embedded page ships as a gzipped PROGMEM blob that is normally
  produced by `npm run build` in embedded/ (webpack, then
  config/buildheader.js). This fork adds an always-on indicator that shows
  who owns the GRBL bus, but the WebUI bundle cannot be rebuilt here because
  Node is not installed.

  Instead this script patches the already built artifact, which needs no
  tooling beyond PowerShell:

    embedded/dist/index.html.gz          (pristine input, never modified)
      -> gunzip
      -> splice a <style> and a <script> before the final </body>
      -> gzip to embedded/dist/index.html.bridge.gz
      -> emit embedded/dist/embedded.h and esp3d/src/modules/http/embedded.h

  The badge is defined by a single string, $BadgeMarkup, which is also
  mirrored into embedded/src/index.js so that a real `npm run build` produces
  the same indicator. Keep the two in step.

  The byte to C array emitter is a faithful port of config/buildheader.js.
  It is verified by reading its own output back: the declared
  tool_html_gz_size and every 0xNN token are parsed out of the generated
  header and compared against the gzip bytes that went in. That check is a
  property of the emitter, so unlike a comparison against the header
  currently on disk it stays valid after a patch has been applied, which is
  what previously made re-running this script, or running it with
  -Unpatch, fail. If the emitter check ever fails the script aborts rather
  than overwriting firmware sources.

  The header currently on disk is then classified as pristine, patched, or
  unrecognised purely to report progress. A mismatch is never fatal, because
  both of those are legitimate states.

  The original blob is left untouched, so this is fully reversible: run with
  -Unpatch, and delete embedded/dist/index.html.bridge.gz.

  KNOWN LIMITATION: the badge lives in the embedded fallback page only.
  handle_root() serves <ESP3D_HOST_PATH>index.html(.gz) from the ESP
  filesystem whenever that file has been uploaded, and that file does not
  contain the badge. Append ?forcefallback=yes to the URL to force the
  embedded page, or rebuild the WebUI with Node so the badge comes from
  embedded/src/index.js.

.NOTES
  A future real `npm run build` discards the badge, because webpack
  regenerates index.html.gz from embedded/src. Re-run this script afterwards
  to reapply it. The durable fix is the copy already present in
  embedded/src/index.js.

.PARAMETER Unpatch
  Regenerate the header from the pristine blob, removing the badge.

.PARAMETER Verify
  Run the emitter self check and report the current state, then exit without
  writing anything.
#>
[CmdletBinding()]
param([switch]$Unpatch, [switch]$Verify)

$ErrorActionPreference = "Stop"

$root      = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$dist      = Join-Path $root "embedded\dist"
$srcTxt    = Join-Path $root "embedded\src"
$pristine  = Join-Path $dist "index.html.gz"
$patched   = Join-Path $dist "index.html.bridge.gz"
$distH     = Join-Path $dist "embedded.h"
$fwH       = Join-Path $root "esp3d\src\modules\http\embedded.h"

# Kept in step with the copy at the end of embedded/src/index.js. The state
# words are the ones HTTP_Server::handle_bridge_status() returns.
$BadgeMarkup = @'
<style id="esp3dBridgeBadgeCss">
#esp3dBridgeBadge{position:fixed;right:10px;bottom:10px;z-index:2147483000;display:none;
padding:6px 12px;border-radius:6px;font:600 13px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
color:#fff;box-shadow:0 2px 8px rgba(0,0,0,.35);pointer-events:none;max-width:60vw}
</style>
<script id="esp3dBridgeBadgeJs">
(function(){var ID="esp3dBridgeBadge",el=null;
function ensure(){if(!el){el=document.getElementById(ID);}if(!el){el=document.createElement("div");el.id=ID;document.body.appendChild(el);}return el;}
var TXT={web:"WEB OWNER - pendant commands refused",
remote:"REMOTE LOCKED - web commands refused",
fault:"BRIDGE FAULT - nothing passes until [ESP431]",
alarm:"ALARM - controller needs $X from its owner",
hold:"FEED HOLD - motion suspended",
resync:"RESYNC - waiting for the controller after a reset",
none:"NO OWNER - both senders refused",
off:"bridge inactive",
unknown:"UNKNOWN - bridge state unavailable"};
var COL={web:"#1b5e20",remote:"#b3261e",fault:"#b3261e",alarm:"#8a5a00",
hold:"#8a5a00",resync:"#8a5a00",none:"#444",off:"#444",unknown:"#444"};
/* Every state except "web" is shown, and only "web" is ever green. A missing
   badge would read as "go ahead", so an unreachable endpoint has to be a
   visible grey UNKNOWN rather than a hidden element. */
function paint(s){var e=ensure();
if(!TXT[s]){s="unknown";}
e.textContent=TXT[s];e.style.background=COL[s];e.style.display="block";}
function poll(){fetch("/bridge",{cache:"no-store"}).then(function(r){
if(!r.ok){throw new Error("http "+r.status);}return r.text();})
.then(function(t){paint((t||"").trim());})
.catch(function(){paint("unknown");});}
ensure();poll();setInterval(poll,1000);})();
</script>
'@

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

function Test-HeaderBytes([byte[]]$a, [byte[]]$b) {
    if ($null -eq $a -or $null -eq $b) { return $false }
    if ($a.Length -ne $b.Length) { return $false }
    for ($i = 0; $i -lt $a.Length; $i++) {
        if ($a[$i] -ne $b[$i]) { return $false }
    }
    return $true
}

# Read the generated header back and confirm it still describes the bytes that
# went in. This validates the emitter itself, so it holds no matter what the
# header on disk currently contains.
function Assert-EmitterRoundTrip([byte[]]$headerBytes, [byte[]]$expect) {
    $text = [System.Text.Encoding]::UTF8.GetString($headerBytes)
    if ($text -notmatch '#define\s+tool_html_gz_size\s+(\d+)') {
        throw "self check FAILED: generated header has no tool_html_gz_size. Refusing to write."
    }
    $declared = [int]$Matches[1]
    if ($declared -ne $expect.Length) {
        throw ("self check FAILED: header declares {0} bytes, blob is {1}. Refusing to write." -f $declared, $expect.Length)
    }
    $m = [regex]::Matches($text, '0x([0-9a-fA-F]{2})')
    if ($m.Count -ne $expect.Length) {
        throw ("self check FAILED: header holds {0} byte tokens, blob is {1}. Refusing to write." -f $m.Count, $expect.Length)
    }
    for ($i = 0; $i -lt $expect.Length; $i++) {
        $got = [Convert]::ToInt32($m[$i].Groups[1].Value, 16)
        if ($got -ne $expect[$i]) {
            throw ("self check FAILED: first difference at byte {0}: header has 0x{1:x2}, blob has 0x{2:x2}. Refusing to write." -f `
                    $i, $got, $expect[$i])
        }
    }
    if ($text -notmatch '#ifndef\s+__embedded_h' -or $text -notmatch '#endif\s+//__embedded_h') {
        throw "self check FAILED: generated header is missing its include guard. Refusing to write."
    }
}

# ---- self check: the emitter must faithfully round trip any blob ----
$pristineBytes = Read-RawBytes $pristine
Assert-EmitterRoundTrip (ConvertTo-EmbeddedHeader $pristineBytes) $pristineBytes
Write-Host ("[ok] emitter round trips {0} gzip bytes exactly" -f $pristineBytes.Length)

$pristineHeader = ConvertTo-EmbeddedHeader $pristineBytes

# ---- build the page to embed ----
if ($Unpatch) {
    $gzOut = $pristine
    Write-Host "[ok] unpatched, using pristine blob"
} else {
    $html = [System.Text.Encoding]::UTF8.GetString((Read-GzipBytes $pristine))
    $marker = "</body>"
    $at = $html.LastIndexOf($marker)
    if ($at -lt 0) { throw "no </body> found in the embedded page" }
    if ($html.Contains($BadgeMarkup)) {
        throw "the pristine blob already contains the badge; it is not pristine. Refusing to write."
    }
    $html = $html.Substring(0, $at) + $BadgeMarkup + $html.Substring($at)
    Write-GzipBytes ([System.Text.Encoding]::UTF8.GetBytes($html)) $patched
    $gzOut = $patched
    Write-Host ("[ok] injected badge, page {0} -> {1} bytes, gzip {2} -> {3} bytes" -f `
        (Read-GzipBytes $pristine).Length, $html.Length, `
        (Get-Item $pristine).Length, (Get-Item $patched).Length)
}

# The header we are about to write must also survive the same check.
$outBytes = Read-RawBytes $gzOut
Assert-EmitterRoundTrip (ConvertTo-EmbeddedHeader $outBytes) $outBytes

# ---- report what the header on disk currently is ----
# Purely informational. Both patched and unpatched are valid states, so this
# must not gate the write the way the old self check did.
if (Test-Path -LiteralPath $fwH) {
    $onDisk = [System.IO.File]::ReadAllBytes($fwH)
    if (Test-HeaderBytes $onDisk $pristineHeader) {
        Write-Host "[--] header on disk is currently unpatched"
    } elseif (Test-Path -LiteralPath $patched) {
        if (Test-HeaderBytes $onDisk (ConvertTo-EmbeddedHeader (Read-RawBytes $patched))) {
            Write-Host "[--] header on disk is currently patched"
        } else {
            Write-Host "[--] header on disk does not match either blob, it will be overwritten"
        }
    } else {
        Write-Host "[--] header on disk does not match the pristine blob, it will be overwritten"
    }
}

if ($Verify) {
    Write-Host "[ok] -Verify, nothing written"
    return
}

# ---- regenerate headers ----
$header = ConvertTo-EmbeddedHeader $outBytes
[System.IO.File]::WriteAllBytes($distH, $header)
[System.IO.File]::WriteAllBytes($fwH, $header)
Write-Host "[ok] wrote embedded\dist\embedded.h and esp3d\src\modules\http\embedded.h"
