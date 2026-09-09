# Bosch Project — CAN trên STM32F405

Bài tập lớn chương trình thực tập Bosch 2026, gồm hai bài thực hành độc lập
trên vi điều khiển STM32F405RGT6 (board Waveshare Open405R-C):

| Thư mục | Bài | Nội dung |
|---|---|---|
| [`diagnostic/`](diagnostic) | Chẩn đoán | UDS (ISO 14229) trên nền CAN-TP (ISO 15765-2) |
| [`communication/`](communication) | Truyền thông | Trao đổi khung định kỳ giữa hai node CAN, có checksum. Có hai bản dựng: [`one-board/`](communication/one-board) (bản cuối, đã nghiệm thu đủ) và [`two-boards/`](communication/two-boards) |

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

Node 1 là phần được chấm điểm. Node 2 đóng vai Verification Board của phòng lab —
gửi dữ liệu định kỳ, và **tự kiểm tra lại** khung mà Node 1 trả về đúng theo cách
board thật sẽ làm, nên nghiệm thu được toàn bộ mà không cần thiết bị của lab.

Repo có hai bản dựng cho bài này. Cả hai đều chạy thật; khác nhau ở chỗ Node 2
nằm ở đâu.

### [`one-board/`](communication/one-board) — bản cuối, đã nghiệm thu đủ 7/7

```
        ┌──────── một board Open405R-C ────────┐
        │  CAN1 ──0x012 mỗi 50 ms──►  CAN2     │
        │  Node1  ◄──0x0A2 mỗi 20 ms── Node2   │
        └──────────────────────────────────────┘
             hai transceiver nối bằng dây, trở 120Ω
```

Hai bộ điều khiển CAN độc lập trên cùng một MCU, nói chuyện qua một bus vật lý
thật. Không phải mô phỏng phần mềm — có transceiver, có dây, có trở đầu cuối,
có định thời bit vi sai thật.

### [`two-boards/`](communication/two-boards) — hai board rời

```
[Node 1 · board 1]  ──0x012 mỗi 50 ms──►  [Node 2 · board 2]
      CAN1              bus CAN 500k             CAN1
                    ◄──0x0A2 mỗi 20 ms──
```

Bản đầu tiên, hai board Open405R-C riêng biệt, mỗi board một node, cả hai dùng
CAN1 (`PA11`/`PA12`). Phần CAN đã nghiệm thu đầy đủ; riêng LCD thì chưa chạy
được ở thời điểm đó.

| Thành phần | Vai trò |
|---|---|
| `can012.c` | Bố cục khung `0x012`/`0x0A2` và hàm checksum |
| `node1.c` | Logic Node 1 — nhận `0x0A2`, dựng và phát `0x012` đúng chu kỳ |
| `node2.c` | Logic Node 2 trên board rời — phát `0x0A2`, thẩm định `0x012` nhận được |
| `node2sim.c` | Bản một-board của Node 2, chạy trên CAN2 cùng MCU. Nhịp 20 ms lấy từ `HAL_GetTick()` chứ không dùng timer phần cứng — TIM2 để dành cho nhịp 50 ms của Node 1, vốn là thứ được chấm |
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

## Bốn lỗi đáng ghi lại

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

**Bus-Off không tự phục hồi, và `hcan.State` không hề hay biết.** CAN2 bị đẩy tới
`TEC=248`, `EPV`, `BOFF` — tức đã tự ngắt khỏi bus. Nhưng `AutoBusOff` đang để
`DISABLE`, nên nó không bao giờ tự quay lại; tệ hơn, trạng thái HAL vẫn là
`HAL_CAN_STATE_LISTENING`, nên đoạn phục hồi chỉ kiểm tra `State` không bắt được
gì cả và bộ điều khiển nằm chết vĩnh viễn. Giờ đoạn đó kiểm tra thêm bit `BOFF`
trong `ESR`.

**Hai sự kiện khác nhau mà chung một nhãn log thì log thành vô dụng.** Dòng
"Node 2 phát đi" và dòng "Node 1 nhận được" từng cùng in nhãn `N2->N1`. Nhìn vào
log thấy hai chiều qua lại đều đặn, rất thuyết phục — trong khi thực tế bus chết
hoàn toàn và mọi dòng đó đều chỉ là một phía tự phát. Mất một vòng gỡ lỗi mới nhận
ra. Nhãn bây giờ nói rõ **ai làm gì**: `N1 sent` / `N1 GOT` / `N2 sent` / `N2 GOT`,
và vài khung nhận đầu tiên luôn in bất kể công tắc trace — câu hỏi "có nhận được
gì không" không được phép là thứ mà cấu hình log có thể giấu đi.

## Xung đột chân, và cách né

Đặc tả có cảnh báo *"Allow to re-init GPIO to switch between CAN transceiver and
LCD"*. Cảnh báo đó dành cho **CAN2**:

```
CAN2_TX  = PB6        LCD_PWM (đèn nền) = PB6      ← trùng
CAN1_TX  = PA12       CAN1_RX = PA11               ← không trùng gì
```

Ở bản hai board, chọn CAN1 ngay từ đầu khiến vấn đề biến mất — Node 2 nằm ở board
khác nên CAN2 không cần dùng tới.

Bản một board thì buộc phải đối mặt. STM32F405 có đúng một cách ánh xạ AF9 thay thế
cho CAN2 (`PB12`=RX, `PB13`=TX), và đó là điều gợi ý "Remap" của trainer nhắm tới.

**Nhưng remap không hoạt động trên board này** — kiểm chứng trên phần cứng:

| Quan sát | Suy ra |
|---|---|
| CAN1 báo `LEC=ack`, không phải `bit-rec` | CAN1 đọc lại đúng từng bit nó phát → vòng TX→transceiver→bus→RX của nó lành lặn, khung thật sự nằm trên bus |
| CAN2 báo `REC=0` mà không nhận được gì | đường RX không phải nhiễu, mà **im lặng tuyệt đối**. Bus có nối nhưng hỏng thì `REC` phải tăng; bus không nối thì `REC` đứng ở 0 |

`REC=0` là chỉ số quyết định. Transceiver CAN2 của board được hàn cứng vào
`PB5`/`PB6` — **remap dời được chân của con chip, không dời được đường mạch.**

Mẹo chẩn đoán: cho bộ điều khiển nghi ngờ chạy `CAN_MODE_SILENT` trước. Nó không
phát được nên không thể sinh lỗi bit và không thể vào bus-off, nên mọi thứ nó báo
về việc *nhận* đều sạch, không lẫn hệ quả của việc phát.

**Cách xử lý, và vì sao xung đột chân hoá ra không thành vấn đề:** trả CAN2 về
`PB5`/`PB6` và để nó dùng chung `PB6` với đèn nền LCD. `CAN2_TX` lúc rỗi ở mức
recessive (cao = đèn sáng), còn một khung 130 bit ở 500 kbit/s chỉ giữ chân đó
trong 260 µs của mỗi chu kỳ 20 ms — đèn nền mờ đi **dưới 1%**, mắt không thấy
được. Chính cái xung đột mà toàn bộ nỗ lực remap muốn né, hoá ra không đáng né.

Cài đặt nằm trong khối `USER CODE` của `HAL_CAN_MspInit()` (công tắc
`NODE2SIM_CAN2_DEFAULT_PINS` trong `node2sim.h`), nên file `.ioc` vẫn giữ nguyên
gán `PB12`/`PB13` và không bao giờ cần generate lại.

Một lưu ý khác về chân: `PA11`/`PA12` đồng thời là `USB_OTG_FS D−/D+`, nên cắm cáp
USB của board vào máy tính sẽ kéo `CAN1_RX` xuống dominant qua điện trở 15 kΩ của
host và làm chết CAN — cấp nguồn qua cổng 5VDC thay vì USB.

## Công tắc biên dịch

| Công tắc | Ý nghĩa |
|---|---|
| `NODE1_DIAG` | Bảng tổng kết 2 giây, `dt=` mỗi lần phát, và bản in thô các thanh ghi CAN |
| `NODE1_TRACE_FRAMES` | In từng khung. `0` chỉ còn dòng `[2s]` — dễ đọc hơn nhiều khi đang chẩn đoán, vì dòng tổng kết dài không còn bị hàng trăm dòng trace đẩy ra khỏi hàng đợi log |
| `NODE1_USE_LCD` | Tắt hẳn phần LCD, để một lỗi màn hình không bị nhầm thành lỗi bus |
| `NODE2SIM_LISTEN_ONLY` | Đặt CAN2 vào `CAN_MODE_SILENT` và ngừng phát. Bộ điều khiển khi đó không thể sinh lỗi bit, không thể vào bus-off — nên những gì nó báo về việc *nhận* là sạch. Đây là công tắc đã phân định được "không nghe được" với "không nói được" |
| `NODE2SIM_CAN2_DEFAULT_PINS` | `1` bỏ qua remap `PB12`/`PB13` và trả CAN2 về `PB5`/`PB6`, nơi transceiver thật sự nằm |
| `LCD_REFRESH_MS` | Nhịp vẽ lại màn. Không phải tuỳ chọn thẩm mỹ: khung về mỗi ~20 ms còn vẽ đủ 8 dòng mất ~29 ms, nên vẽ theo mỗi khung sẽ khởi động lại lượt vẽ trước khi nó kịp xong, và những dòng không bao giờ vẽ tới lại đúng là những dòng mới nhất |
| `LCD_TEST_PATTERN` / `LCD_TEST_BACKLIGHT` / `LCD_TEST_PINSCAN` | Ba mức kiểm tra màn hình khi đưa vào hoạt động lần đầu, thu hẹp dần từ "panel có chạy không" tới "chân nào điều khiển đèn nền" |

## Trạng thái nghiệm thu

Bản `one-board/` đã chạy đủ **7/7** tiêu chí chấm phần code, trên bus CAN thật
(`BTR=01290005` — không có bit SILM/LBKM nên đúng là chế độ normal):

```
[2s] TX=40 RX=100  dt=50..50 ms  CAN1 LEC=ok TEC=0 REC=0
     dbg isr=100 getErr=0 lastId=0A2 txErr=0  TSR=1C000003 BTR=01290005
[2s] N2 TX=100 pass=40 fail=0  CAN2 LEC=ok TEC=0 REC=0
```

| Tiêu chí | Bằng chứng |
|---|---|
| Phát `0x012` | `TX=40` mỗi 2 giây = 2000 ÷ 50 ms |
| Đúng hạn 50 ms ±1 ms | `dt=50..50 ms` — nhỏ nhất bằng lớn nhất, sai lệch **0 ms** |
| Byte 0/1 = giá trị nhận được | Node 2 đối chiếu với lịch sử nó đã gửi |
| Byte 2 = tổng | `2C+11=3D`, `2E+15=43`, `31+1B=4C`… kiểm tay đều khớp |
| Checksum byte 6 | Node 2 tự tính lại độc lập, `fail=0` |
| Hiển thị UART | log ở trên |
| **Hiển thị log lên LCD** | 8 dòng đọc được trên màn, xen kẽ `CAn1`/`CAn2` |

`RX=100` là các khung 20 ms từ Node 2 dội về; `TSR` có `TXOK0=1` nên khung phát đi
được ACK thật sự, không phải chỉ nạp được vào mailbox.

Bản `two-boards/` đã nghiệm thu phần CAN từ trước với cùng kết quả
(`dt=50..50 ms`, `pass=40 fail=0`), nhưng LCD thì chưa chạy được ở thời điểm đó.

### Về phần LCD

Màn hình im lặng suốt một thời gian dài, và **nguyên nhân không nằm ở driver**.
Bộ lệnh khởi tạo, ánh xạ chân và đường vẽ DMA đều đúng ngay từ đầu — sau này đối
chiếu với datasheet ST7789V gốc của Sitronix thì từng mã lệnh đều khớp, và tài
liệu cũng không hề đòi hỏi trình tự power/gamma bắt buộc nào.

Có một bảng lệnh lưu truyền trên mạng với `B1/B2/B3` (frame rate) và `C0`–`C4`
(power control) — **đó là của ST7735**, một con chip khác. Đưa bộ lệnh đó vào một
ST7789V thật chỉ ghi giá trị vô nghĩa vào những thanh ghi không liên quan.

---

## Phạm vi repo

Chỉ chứa mã nguồn tự viết và mô tả các bản vá. **Không** bao gồm tài liệu đặc tả,
công cụ hay bộ khung do Bosch cung cấp.

Các file HAL driver, CMSIS, linker script và cấu hình project cũng không đưa vào
vì thuộc bộ khung gốc — cần lấy từ gói tài liệu của chương trình để build được.
