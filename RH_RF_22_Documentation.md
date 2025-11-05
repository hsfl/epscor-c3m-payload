# RH_RF22 — Driver Cheat‑Sheet (RadioHead)

**Use this when the agent doesn’t have the library files.** This summarizes the real `RH_RF22.h` surface area and the Si443x/RFM22B quirks.

For this project we USE the RFM23BP Radio.

---

## What it wraps
- Silicon Labs **Si4430/31/32** / HopeRF **RFM22/23** FSK/GFSK/OOK transceivers.
- SPI + **NIRQ** (interrupt). Optional **SDN**. GPIO0/GPIO1 may drive the antenna switch.

## Key compile‑time constants
- `RH_RF22_MAX_MESSAGE_LEN` (default overridden to **50** here; 1‑byte length allows up to 255)
- `RH_RF22_FIFO_SIZE = 64`
- FIFO thresholds: TX **AEM=4**, RX **FULL=55**

## Useful register bits (names match header)
- **INT status**: `REG_03/04` (read‑to‑clear): `IPKSENT`, `IPKVALID`, `ICRCERROR`, etc.
- **INT enable**: `REG_05/06`.
- **Mode**: `REG_07` (`TXON`, `RXON`, `PLLON`, `XTON`, `SWRES`).
- **Mode2**: `REG_08` (`FFCLRTX`, `FFCLRRX`, `AUTOTX`, `RXMPK`).
- **Pkt ctrl**: `REG_30` (CRC on/off, polynomial `CRC[1:0]`).
- **Header ctrl**: `REG_32/33` (broadcast/header check, fixed/var length).
- **Preamble/sync**: `REG_34–39`.
- **Power**: `REG_6D` (`TXPOW_*`, RF23B/BP variants).
- **Modem**: `REG_6E–72`, `REG_1C`, `REG_20–25`, `REG_69` (AFC/AGC, rate, deviation, IF BW).
- **Freq**: `REG_75–77` (+ optional hop `REG_79/7A`).

---

## Class surface (public)
**Construction**
```cpp
RH_RF22(uint8_t ss=SS, uint8_t nirq=2, RHGenericSPI& spi=hardware_spi);
```
Multiple instances supported (up to 3) with unique NIRQ lines.

**Bring‑up & status**
- `bool init();` (defaults: 434.0 MHz, `FSK_Rb2_4Fd36`, attaches ISR)
- `void reset();`
- `uint8_t statusRead();` (REG_02)
- `uint8_t ezmacStatusRead();`
- `uint8_t rssiRead();` (instantaneous, **not dBm**; for last‑packet dBm use base‑class `lastRssi()`)
- `uint32_t getLastPreambleTime();`

**RF setup**
- `bool setFrequency(float centreMHz, float afcPullIn=0.05);`
- `bool setFHStepSize(uint8_t fhs10kHz);` / `bool setFHChannel(uint8_t ch);`
- `void setTxPower(uint8_t level /* RH_RF22_TXPOW_* */);`
- `void setPreambleLength(uint8_t nibbles /* 4‑bit each */);`
- `void setSyncWords(const uint8_t* words, uint8_t len /*1..4*/);`
- `void setModemRegisters(const ModemConfig* m);`
- `bool setModemConfig(ModemConfigChoice idx);`
- `bool setCRCPolynomial(CRCPolynomial poly /* CCITT, 16‑IBM (default), IEC‑16, Biacheva */);`
- `void setGpioReversed(bool reversed=false);` (swap TX_ANT/RX_ANT wiring quirk)
- `void setOpMode(uint8_t modeBits);` / `void setIdleMode(uint8_t modeBits);`

**TX/RX**
- `bool send(const uint8_t* data, uint8_t len);`  (len>0)
- `bool available();`
- `bool recv(uint8_t* buf, uint8_t* len);`
- `void setModeIdle();` `void setModeRx();` `void setModeTx();`
- Base‑class waits: `waitPacketSent([timeout])`, `waitAvailableTimeout(timeout)`, etc.
- `virtual bool sleep();`

**Headers & addressing**
- Uses standard RadioHead headers (TO, FROM, ID, FLAGS). `setPromiscuous(true)` accepts any TO.

**Platform note**
- ESP8266 overrides `waitPacketSent()` and exposes `loopIsr()` (move SPI from ISR to loop context).

---

## Modem presets (`ModemConfigChoice`)
A curated subset—match both ends:
- **FSK**: `FSK_Rb2Fd5`, `FSK_Rb9_6Fd45`, `FSK_Rb38_4Fd19_6`, `FSK_Rb125Fd125`, `FSK_Rb_512Fd2_5` (POCSAG), …
- **GFSK**: `GFSK_Rb2Fd5`, `GFSK_Rb57_6Fd28_8`, `GFSK_Rb125Fd125`, …
- **OOK**: `OOK_Rb1_2Bw75`, `OOK_Rb9_6Bw335`, `OOK_Rb38_4Bw335`, `OOK_Rb40Bw335`.
- **UnmodulatedCarrier** for test tones; `FSK_PN9_Rb2Fd5` for PN9 test.

---

## Power levels (REG_6D)
- **RFM22B**: `TXPOW_1/2/5/8/11/14/17/20 dBm`
- **RFM23B**: `RF23B_TXPOW_*` (‑8…+13 dBm)
- **RFM23BP**: `RF23BP_TXPOW_28/29/30 dBm` (requires robust 5V rail; PA drive caveats)

---

## Packet format (actual)
```
[PREAMBLE 4B][SYNC 2B default 0x2D,0xD4]
[TO][FROM][ID][FLAGS][LENGTH][DATA…][CRC16(IBM)]
```
- `LENGTH` is **payload length** (0..255). Driver caps by `maxMessageLength()`.

---

## Typical flows
**Init**
```cpp
RH_RF22 rf22(SS, NIRQ);
if (!rf22.init()) fail();
rf22.setFrequency(915.0);
rf22.setModemConfig(RH_RF22::GFSK_Rb38_4Fd19_6);
rf22.setTxPower(RH_RF22_TXPOW_11DBM);
```
**TX → RX (ping/pong)**
```cpp
rf22.send(data, len);
rf22.waitPacketSent(100);
rf22.setModeRx();
if (rf22.waitAvailableTimeout(50)) { uint8_t n=sizeof(buf); if (rf22.recv(buf,&n)) { /*…*/ } }
```

---

## FIFO & interrupt hygiene
**Read both** `INT_STATUS1/2` to clear. If things wedge:
```cpp
rf22.setModeIdle();
rf22.spiWriteRegister(RH_RF22_REG_08_OPERATING_MODE2,
                      RH_RF22_FFCLRTX | RH_RF22_FFCLRRX);
(void)rf22.spiReadRegister(RH_RF22_REG_03_INTERRUPT_STATUS1);
(void)rf22.spiReadRegister(RH_RF22_REG_04_INTERRUPT_STATUS2);
rf22.setModeRx();
```
When to do this: CRC storms, aborted TX, or `waitPacketSent()` timeouts.

---

## Antenna switch gotcha
Some boards wire **GPIO0↔RX_ANT** and **GPIO1↔TX_ANT** reversed. Call `setGpioReversed(true)` after `init()` if you see TX stuck / RX deaf.

---

## Reliability tips
- Match **frequency, modem preset, preamble, sync, CRC** across nodes.
- Keep messages **≤ `maxMessageLength()`** (consider your build‑time override of 50).
- After TX expecting a reply, **force `setModeRx()`**.
- For ack/retry, pair with `RHReliableDatagram` or mirror its pattern (ID‑based acks with timeouts).

---

## Quick answers
- **Why `send()` false?** len==0 or (if CAD/LBT used upstream) channel busy; or state jam → do FIFO reset.
- **RSSI units?** `rssiRead()` is chip units; `lastRssi()` (base class) gives dBm for last packet.
- **Sleep?** `sleep()` lowers power; wake by entering Idle/Tx/Rx (adds wake latency).

*That’s the RH_RF22 essentials. Keep this short; defer fine register math to the header or datasheet when needed.*

