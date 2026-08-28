# Patch: serial_manager.py — hien thi dap ung UDS dang Hex

## Van de

`_read_loop()` cua tool goc phan loai du lieu nhan duoc chi dua tren viec
byte co in duoc hay khong:

```python
if all(byte in (9, 10, 13) or 32 <= byte <= 126 for byte in data):
    # -> hien thi dang String
```

Dap ung duong cua dich vu $2E chi gom dung mot byte `0x6E`. Vi 0x6E nam trong
vung ky tu ASCII in duoc (110 = chu 'n'), tool hien thi thanh:

```
RX: [String] n
```

khien khong the doi chieu truc tiep voi dac ta (dac ta ghi 0x6E).

## Cach sua

Dong log dang chu do firmware gui ra luon ket thuc bang ky tu xuong dong,
con dap ung UDS la nhi phan tho thi khong. Dung dac diem nay de phan biet:

```python
# A device log line is text and always ends with a newline. A UDS
            # response is raw binary and never does, so it must stay in hex -
            # otherwise a one-byte positive response such as 0x6E ($2E) would
            # be rendered as the letter "n" and no longer match the spec.
            is_text_log = data.endswith(b"\n") and all(
                byte in (9, 10, 13) or 32 <= byte <= 126 for byte in data
            )

            if is_text_log:
                text = data.decode("utf-8", errors="replace")
                lines = text.splitlines() or [text]
                for line in lines:
                    self._logger.info("RX: [String] %s", line)
            else:
                self._logger.info("RX: [Hex] %s", data.hex(" ").upper())
```

## Ket qua

| Du lieu | Truoc | Sau |
|---|---|---|
| `6E` (dap ung $2E) | `[String] n` | `[Hex] 6E` |
| `62 01 23 07 12` | `[Hex] ...` | `[Hex] ...` (khong doi) |
| `[DBG] C1esr=...
` | `[String] ...` | `[String] ...` (khong doi) |
