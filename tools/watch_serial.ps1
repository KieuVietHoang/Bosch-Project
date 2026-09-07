# Doc lien tuc cong COM va in ra console - nhe hon Hercules khi du lieu chay nhanh.
# Dong Hercules truoc khi chay cai nay (chi 1 chuong trinh giu duoc cong COM).
# Dung: Ctrl+C de thoat.

param(
    [string]$PortName = "COM11",
    [int]$BaudRate = 115200
)

$port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 500
$port.Open()

Write-Host "Da mo $PortName @ $BaudRate. Ctrl+C de thoat." -ForegroundColor Green

try {
    while ($true) {
        try {
            $line = $port.ReadLine()
            Write-Host $line.TrimEnd("`r")
        }
        catch [System.TimeoutException] {
            # khong co du lieu trong 500ms - binh thuong, thu lai
        }
    }
}
finally {
    $port.Close()
    Write-Host "Da dong cong." -ForegroundColor Yellow
}
