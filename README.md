# Bosch Project — UDS Diagnostic over CAN (STM32F405)

Bài tập lớn chương trình thực tập Bosch 2026: triển khai lớp chẩn đoán
UDS (ISO 14229) trên nền CAN-TP (ISO 15765-2) cho vi điều khiển STM32F405RGT6.

## Kiến trúc

Cả máy chẩn đoán lẫn ECU đều được mô phỏng bằng phần mềm trên **cùng một board**,
nối với nhau qua hai ngoại vi CAN và một bus vật lý (2 transceiver + điện trở đầu cuối):

```
PC ──UART3──► [CAN1 · vai Tester] ══ bus CAN ══ [CAN2 · vai ECU]
   khung BEA        chuyển tiếp        0x712 / 0x7A2      xử lý UDS
```

| Thành phần | Vai trò | Có hiểu UDS? |
|---|---|---|
| `bea_tester.c` | Cầu nối UART ↔ CAN. Dò khung BEA, đóng gói CAN-TP, chuyển tiếp hai chiều | Không |
| `can_tp.c` | Lớp vận chuyển — đóng/mở gói Single Frame, đệm `0x55`, DLC cố định 8 | Không |
| `dcm.c` | Bộ điều phối — đọc SID, gọi handler tương ứng, sinh đáp ứng âm | Có |
| `dcm_rdbi.c` | `$22` ReadDataByIdentifier | Có |
| `dcm_seca.c` | `$27` SecurityAccess — seed/key, máy trạng thái, hẹn giờ | Có |
| `dcm_wdbi.c` | `$2E` WriteDataByIdentifier | Có |

Tách lớp theo nguyên tắc: lớp vận chuyển không biết ý nghĩa dữ liệu, lớp ứng dụng
không chạm vào thanh ghi CAN.

## Dịch vụ đã triển khai

| Dịch vụ | Chức năng | Yêu cầu bảo mật |
|---|---|---|
| `$22` | Đọc CAN ID hiện hành của Tester | Không |
| `$22` | Đọc nhiệt độ từ cảm biến nội qua ADC1 | Không |
| `$27` | Mở khoá bằng seed/key, báo hiệu bằng LED0 (PB0) trong 5 giây | — |
| `$2E` | Ghi CAN ID mới, áp dụng sau chu kỳ Ignition (nút User) | Có — mức 1 |

Kèm xử lý đáp ứng âm: sai độ dài (`0x13`), DID không hỗ trợ (`0x31`),
chưa mở khoá (`0x33`), khoá sai (`0x35`) — khoá sai còn kèm hình phạt 10 giây.

## Hiệu chỉnh so với bộ khung được cấp

| Hạng mục | Trước | Sau | Lý do |
|---|---|---|---|
| Bit timing CAN | 117 647 bit/s | **500 000 bit/s** | Đáp ứng yêu cầu tốc độ bus. `Prescaler=6, BS1=10TQ, BS2=3TQ, SJW=2TQ` → NBT 14 TQ, điểm lấy mẫu 78,6 % |
| Bộ lọc CAN | `FilterActivation = 0` | Cấu hình đầy đủ, `ENABLE` | Bộ khung gốc để cấu trúc lọc chưa khởi tạo nên mọi khung bị chặn |
| `SlaveStartFilterBank` | không đặt | `14` cho cả hai CAN | CAN2 là slave, bắt buộc dùng bank ≥ ranh giới này |
| GPIO PB0 | chưa cấu hình | ngõ ra push-pull | LED0 báo trạng thái mở khoá, bộ khung gốc thiếu |
| `PrintCANLog()` | `sprintf` ghi tràn | mở rộng bộ đệm, dùng `%03X` | Hai lỗi tràn bộ đệm do trình biên dịch cảnh báo, cộng một lỗi thiếu ký tự kết thúc chuỗi mà trình biên dịch không bắt |
| `HAL_UART_RxCpltCallback` | dồn byte vào mảng 4096, không bao giờ xoá | nạp vào máy trạng thái dò khung | Tránh tràn bộ đệm sau 4096 byte |

## Nguyên tắc xử lý ngắt

Các hàm phục vụ ngắt chỉ chép dữ liệu và bật cờ. Mọi thao tác nặng — phát khung CAN,
gửi UART — đẩy sang vòng lặp chính, tránh gọi hàm chặn trong ngắt.

## Chế độ chẩn đoán

`bea_tester.h` có cờ biên dịch `BEA_DEBUG_TRACE`. Khi bật, firmware in trạng thái
thanh ghi lỗi của cả hai bộ điều khiển CAN cùng bộ đếm khung nhận được — dùng để
khoanh vùng sự cố khi không có mạch gỡ lỗi phần cứng:

```
[DBG] REQ 3B -> id=712 TX=OK
[DBG] C1esr=00000000 C2esr=00000000 rx1=1 rx2=1
```

Đặt về `0` khi chạy chính thức để đường UART chỉ còn khung giao thức.

## Phạm vi repo

Chỉ chứa mã nguồn tự viết và mô tả các bản vá. **Không** bao gồm tài liệu đặc tả,
công cụ hay bộ khung do Bosch cung cấp.

Các file HAL driver, CMSIS, linker script và cấu hình project cũng không đưa vào
vì thuộc bộ khung gốc — cần lấy từ gói tài liệu của chương trình để build được.
