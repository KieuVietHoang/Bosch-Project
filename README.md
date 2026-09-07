# Bosch Project — CAN trên STM32F405

Bài tập lớn chương trình thực tập Bosch 2026, gồm hai bài thực hành độc lập
trên vi điều khiển STM32F405RGT6 (board Waveshare Open405R-C):

| Thư mục | Bài | Nội dung |
|---|---|---|
| [`diagnostic/`](diagnostic) | Chẩn đoán | UDS (ISO 14229) trên nền CAN-TP (ISO 15765-2) |
| [`communication/`](communication) | Truyền thông | Trao đổi khung định kỳ giữa hai node CAN, có checksum |

Không có mạch gỡ lỗi phần cứng (ST-Link) trong suốt quá trình — mọi việc chẩn đoán
đều qua UART, nên phần lớn công sức nằm ở chỗ làm cho firmware **tự nói ra được
nó đang sai ở đâu**.

---

# Bài 1 — Chẩn đoán UDS

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

---

# Bài 2 — Truyền thông CAN

Hai board Open405R-C rời, mỗi board một node, nối với nhau bằng một bus CAN thật.
**Cả hai đều dùng CAN1 (`PA11`/`PA12`)** — xem phần xung đột chân bên dưới.

```
[Node 1 · board 1]  ──0x012 mỗi 50 ms──►  [Node 2 · board 2]
      CAN1              bus CAN 500k             CAN1
                    ◄──0x0A2 mỗi 20 ms──
```

Node 1 là phần được chấm điểm. Node 2 đóng vai Verification Board của phòng lab —
gửi dữ liệu định kỳ, và **tự kiểm tra lại** khung mà Node 1 trả về đúng theo cách
board thật sẽ làm, nên có thể nghiệm thu toàn bộ mà không cần thiết bị của lab.

| Thành phần | Vai trò |
|---|---|
| `can012.c` | Bố cục khung `0x012`/`0x0A2` và hàm checksum |
| `node1.c` | Logic Node 1 — nhận `0x0A2`, dựng và phát `0x012` đúng chu kỳ |
| `node2.c` | Logic Node 2 — phát `0x0A2`, thẩm định `0x012` nhận được |
| `lcd.c` | Driver ST7789 qua SPI1, vẽ bằng DMA |
| `uart_log.c` | Nhật ký UART không chặn — hàng đợi vòng, DMA rút |

## Checksum: tên gọi không khớp với thuật toán

Đặc tả gọi nó là "CRC 8 SAE J1850", nhưng đoạn mã tham chiếu nhúng trong tài liệu
**không** khớp tham số J1850 chuẩn. Đối chiếu bằng cách biên dịch chính đoạn mã đó
và chạy với hai vector kiểm thử in trong tài liệu:

| Cách tính | `{2A 0D 00 A3 22 00 6C}` | `{A2 5A FE 9F 8C 04 10}` | |
|---|---|---|---|
| init `0x00`, không XOR ra, **duyệt byte ngược** | `0xB4` | `0xA3` | ✅ khớp mã tham chiếu |
| J1850 sách vở (init `0xFF`, XOR ra `0xFF`, xuôi) | `0xBA` | `0x6B` | ❌ |
| init `0x00`, không XOR ra, xuôi | `0xB0` | `0x61` | ❌ |

Dùng thư viện CRC-8/SAE-J1850 có sẵn sẽ ra sai. Đây là loại chi tiết chỉ lộ ra khi
kiểm chứng lại đặc tả thay vì tin vào tên gọi.

## Ràng buộc thời gian và cái giá của việc in log

Yêu cầu: khung `0x012` phải phát **mỗi 50 ms ±1 ms**. TIM2 lo phần đó và không bao
giờ trôi. Nhưng thời điểm khung thực sự lên bus lại phụ thuộc vào lúc vòng lặp
chính kịp nạp mailbox — nên **mọi thứ chặn CPU đều ăn vào ngân sách này**.

Đo được trên phần cứng thật: với `HAL_UART_Transmit()` chặn (~3–4 ms mỗi dòng) và
bật in đầy đủ ~90 dòng/giây, chu kỳ phát **dao động 48/52 ms** thay vì phẳng 50 ms.

Cách xử lý — cả hai đường ra đều chuyển sang DMA:

| Đường ra | DMA | Chi phí CPU mỗi vòng lặp |
|---|---|---|
| Nhật ký UART | DMA1 Stream3 (USART3_TX) | chép vào hàng đợi vòng rồi thoát |
| Điểm ảnh LCD | DMA2 Stream3 (SPI1_TX) | khởi động một lượt burst rồi thoát |

Thêm hai nguyên tắc trong `Node1_Periodic()`: nạp mailbox CAN **trước** mọi việc
log/LCD, và việc vẽ LCD được chia nhỏ theo máy trạng thái để mỗi lần gọi chỉ tốn
vài micro giây.

Kết quả sau khi sửa, vẫn đang in đầy đủ ~90 dòng/giây:

```
[2s] TX=40 RX=100  dt=50..50 ms  CAN1 LEC=ok TEC=0 REC=0
[2s] N2 got=40 pass=40 fail=0    CAN2 LEC=ok TEC=0 REC=0
```

`dt=50..50 ms` — chênh lệch giữa nhỏ nhất và lớn nhất bằng 0.

## Hai lỗi đáng ghi lại

**bxCAN kẹt trong init mode.** Mọi `HAL_CAN_AddTxMessage()` trả lỗi trong khi các
thanh ghi lỗi CAN sạch bong và phần còn lại của firmware chạy hoàn hảo. Dấu hiệu
nhận biết: `MSR` bit 0 (`INAK`) bằng 1 trong khi cả ba mailbox đều trống. Khối CAN
chỉ rời init mode sau khi thấy **11 bit recessive liên tiếp** trên CANRX; nguyên
nhân gốc là chân `PB6` (CAN2_TX) bị thả nổi khiến transceiver CAN2 ghì bus xuống
dominant. Sửa bằng cách ép `PB6` lên mức recessive ngay trong `MX_GPIO_Init()`.

**DMA TX của UART cần cả ngắt của USART.** Ở chế độ DMA normal (không circular),
`UART_DMATransmitCplt()` **không** gọi `HAL_UART_TxCpltCallback` — nó chỉ bật cờ
`USART_CR1_TCIE` rồi đợi ngắt Transmit-Complete của chính USART. Dự án đã xoá
`USART3_IRQHandler` từ hồi TX còn chặn, nên callback không bao giờ chạy và chỉ
đúng **một dòng đầu tiên** được gửi đi. `SPI_DMATransmitCplt` thì **không** đối
xứng — nó gọi callback trực tiếp, nên đường LCD không cần ngắt tương ứng.

## Xung đột chân, và cách né

Đặc tả có cảnh báo *"Allow to re-init GPIO to switch between CAN transceiver and
LCD"*. Cảnh báo đó dành cho **CAN2**:

```
CAN2_TX  = PB6        LCD_PWM (đèn nền) = PB6      ← trùng
CAN1_TX  = PA12       CAN1_RX = PA11               ← không trùng gì
```

Chọn CAN1 ngay từ đầu khiến vấn đề biến mất hoàn toàn, không cần chuyển đổi GPIO
qua lại. Cần lưu ý thêm: `PA11`/`PA12` đồng thời là `USB_OTG_FS D−/D+`, nên cắm
cáp USB của board vào máy tính sẽ kéo `CAN1_RX` xuống dominant qua điện trở 15 kΩ
của host và làm chết CAN — cấp nguồn qua cổng 5VDC thay vì USB.

## Công tắc biên dịch

| Công tắc | Ý nghĩa |
|---|---|
| `NODE1_SELFTEST` | `0` chạy thật; `1` chạy loopback với Node 2 giả lập bằng phần mềm, không cần board thứ hai |
| `NODE1_DIAG` | Bảng tổng kết 2 giây, `dt=` mỗi lần phát, và bản in thô các thanh ghi CAN |
| `NODE1_TRACE_FRAMES` | In từng khung, có nhãn rõ node nào gửi cho node nào |
| `NODE1_USE_LCD` | Tắt hẳn phần LCD |
| `LCD_TEST_PATTERN` / `LCD_TEST_BACKLIGHT` / `LCD_TEST_PINSCAN` | Ba mức kiểm tra màn hình khi đưa vào hoạt động lần đầu, thu hẹp dần từ "panel có chạy không" tới "chân nào điều khiển đèn nền" |

## Trạng thái nghiệm thu

Đã kiểm chứng trên **hai board thật, bus CAN thật** (`BTR=01290005` — không có bit
SILM/LBKM nên đúng là chế độ normal):

- Khung `0x012` phát đúng chu kỳ 50 ms, sai lệch 0 ms
- Node 2 nhận đủ 40/40 khung mỗi cửa sổ 2 giây và **thẩm định đúng toàn bộ**:
  byte0/1 khớp giá trị nó đã gửi, byte2 = tổng, checksum byte 6, các byte đệm bằng 0
- Chiều ngược lại chạy đều 20 ms
- Không một lỗi bus nào trên cả hai bộ điều khiển

**Chưa nghiệm thu:** phần hiển thị LCD. Driver đã viết xong và build sạch, nhưng
module màn hình trên board thử nghiệm không lên được đèn nền — đây là vấn đề phần
cứng nằm ngoài phạm vi firmware, chưa loại trừ được nên chưa dám coi là đã xong.

---

## Phạm vi repo

Chỉ chứa mã nguồn tự viết và mô tả các bản vá. **Không** bao gồm tài liệu đặc tả,
công cụ hay bộ khung do Bosch cung cấp.

Các file HAL driver, CMSIS, linker script và cấu hình project cũng không đưa vào
vì thuộc bộ khung gốc — cần lấy từ gói tài liệu của chương trình để build được.
