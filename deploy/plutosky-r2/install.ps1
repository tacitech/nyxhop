# NyxHop for the PlutoSky R2: copies pluto.frm onto the board's PlutoSDR drive, ejects it and
# waits for the board to come back with the new firmware. Run install.bat.
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$frm = Join-Path $here "pluto.frm"
if (-not (Test-Path $frm)) { Write-Host "pluto.frm is not next to this script."; exit 1 }

function Find-Drive {
    Get-CimInstance Win32_LogicalDisk | Where-Object { $_.VolumeName -eq "PlutoSDR" } | Select-Object -First 1
}
function Wait-Drive($want, $seconds) {
    for ($i = 0; $i -lt $seconds; $i++) {
        $d = Find-Drive
        if ($want -and $d) { return $d }
        if (-not $want -and -not $d) { return $true }
        Start-Sleep 1
    }
    return $null
}

Write-Host "Looking for the PlutoSDR drive..."
$d = Wait-Drive $true 60
if (-not $d) {
    Write-Host "No PlutoSDR drive after a minute."
    Write-Host "Is the board's USB port plugged in (not the DEBUG port in the middle)? Does it have enough power? Try both Type-C ports."
    exit 1
}
$drive = $d.DeviceID
# a PlutoSDR shows the same drive: its info.html names a Zynq-7010, the R2's a Zynq-7020
$info = "$drive\info.html"
if ((Test-Path $info) -and -not ((Get-Content $info -Raw) -match 'Z7020')) {
    Write-Host "The board on $drive is not a PlutoSky R2 (its model is not a Zynq-7020). This firmware is for the R2 only: nothing was copied."
    exit 1
}
Write-Host "Found it on $drive. Copying the firmware..."
Copy-Item $frm "$drive\pluto.frm" -Force
$a = (Get-FileHash $frm -Algorithm MD5).Hash
$b = (Get-FileHash "$drive\pluto.frm" -Algorithm MD5).Hash
if ($a -ne $b) { Write-Host "The copy came out different. Run this again."; exit 1 }

Write-Host "Copied. Ejecting the drive - the board writes the firmware now. Don't unplug it."
$shell = New-Object -ComObject Shell.Application
$shell.NameSpace(17).ParseName("$drive\").InvokeVerb("Eject")
$null = Wait-Drive $false 120

Write-Host "Waiting for the board to restart (one to three minutes)..."
$d = Wait-Drive $true 360
if (-not $d) {
    Write-Host "The drive did not come back in six minutes."
    Write-Host "Unplug the board and plug it in again. If a file named FAILED... is on the drive, the copy was damaged: run this again."
    Write-Host "If there is no drive at all, see 'Install guide.pdf', section 8."
    exit 1
}
Write-Host "The board is back. NyxHop restarts it once more the first time - a minute or so."
if (Wait-Drive $false 90) {
    $null = Wait-Drive $true 240
}
Write-Host ""
Write-Host "Done. The firmware is on the board."
foreach ($addr in @("192.168.1.10", "192.168.2.1")) {
    try {
        $c = New-Object Net.Sockets.TcpClient
        $ok = $c.BeginConnect($addr, 7202, $null, $null).AsyncWaitHandle.WaitOne(2000)
        if ($ok -and $c.Connected) { Write-Host "NyxHop answers at $addr."; $c.Close(); break }
        $c.Close()
    } catch { }
}
Write-Host "Open the NyxHop app, choose the role for this end and enter the board's address (192.168.1.10 over Ethernet)."
