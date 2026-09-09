# Doc lien tuc cong COM va in ra console.
#
# Nhe hon Hercules rat nhieu khi firmware in ~90 dong/giay - Hercules dung
# giao dien do hoa cho tung dong nen bi treo, con cai nay chi ghi text.
#
# Dung:
#   .\watch_serial.ps1                  # tu do cong (bo qua Bluetooth)
#   .\watch_serial.ps1 -PortName COM11  # chi dinh cong
#   .\watch_serial.ps1 -LogFile log.txt # vua in vua luu ra file
#
# Ctrl+C de thoat.
#
# LUU Y: chi mot chuong trinh giu duoc cong COM. Dong Hercules va
# FLASHER-STM32 truoc khi chay cai nay.
# Parity phai la None - luc NAP thi bootloader dung EVEN, nhung luc CHAY
# thi ung dung dung None.

param(
    [string]$PortName = "",
    [int]$BaudRate = 115200,
    [string]$LogFile = ""
)

# --- Tim cong neu khong duoc chi dinh -------------------------------------
if ([string]::IsNullOrWhiteSpace($PortName)) {
    # Bo qua "Standard Serial over Bluetooth link" - may nao cung co vai cai
    # nhu vay va chung khong bao gio la mach USB-UART.
    $candidates = Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.Name -match '\(COM\d+\)' -and $_.Name -notmatch 'Bluetooth' }

    if ($candidates.Count -eq 0) {
        Write-Host "Khong tim thay cong COM nao (ngoai Bluetooth)." -ForegroundColor Red
        Write-Host "Kiem tra: mach USB-UART da cam vao may chua?" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Cac cong dang co:" -ForegroundColor DarkGray
        [System.IO.Ports.SerialPort]::GetPortNames() | ForEach-Object { Write-Host "  $_" }
        exit 1
    }

    if ($candidates.Count -gt 1) {
        Write-Host "Tim thay nhieu cong, hay chi dinh bang -PortName:" -ForegroundColor Yellow
        $candidates | ForEach-Object { Write-Host "  $($_.Name)" }
        exit 1
    }

    $PortName = [regex]::Match($candidates[0].Name, 'COM\d+').Value
    Write-Host "Tu do duoc: $($candidates[0].Name)" -ForegroundColor DarkGray
}

# --- Mo cong --------------------------------------------------------------
$port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate,
        ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 500

try {
    $port.Open()
}
catch {
    Write-Host "Khong mo duoc ${PortName}: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host "Thuong la do Hercules hoac FLASHER-STM32 dang giu cong." -ForegroundColor Yellow
    exit 1
}

Write-Host "Da mo $PortName @ $BaudRate 8-N-1. Ctrl+C de thoat." -ForegroundColor Green
if ($LogFile) { Write-Host "Dang luu vao $LogFile" -ForegroundColor DarkGray }

# To mau vai tu khoa de doc nhanh - FAIL va STUCK la thu can thay ngay.
try {
    while ($true) {
        try {
            $line = $port.ReadLine().TrimEnd("`r")

            if     ($line -match 'FAIL|STUCK|Error') { Write-Host $line -ForegroundColor Red }
            elseif ($line -match 'PASS')             { Write-Host $line -ForegroundColor Green }
            elseif ($line -match '^\[2s\]|^init:')   { Write-Host $line -ForegroundColor Cyan }
            else                                     { Write-Host $line }

            if ($LogFile) { Add-Content -Path $LogFile -Value $line -Encoding utf8 }
        }
        catch [System.TimeoutException] {
            # Khong co du lieu trong 500ms - binh thuong, thu lai.
        }
    }
}
finally {
    $port.Close()
    Write-Host "Da dong cong." -ForegroundColor Yellow
}
