# Roadio Firmware - Setup & Build Instructions

## Quick Start

This guide will help you get the Roadio firmware building and running on your Nucleo-H723ZG board.

---

## Prerequisites

### 1. Install Zephyr Dependencies

Follow the official Zephyr getting started guide for your OS:
- **Documentation**: https://docs.zephyrproject.org/latest/develop/getting_started/index.html

**Required tools:**
- CMake (≥ 3.20.0)
- Python 3 (≥ 3.8)
- Device Tree Compiler (dtc)
- Zephyr SDK or ARM GCC toolchain

### 2. Install West (Zephyr's meta-tool)

```bash
pip install west
```

### 3. Install Zephyr SDK

```bash
cd zephyr
west sdk install
```

---

## Project Setup

### Your workspace is already initialized!

This repository has already been set up as a west workspace with Zephyr v4.3.0.

**Current structure:**
```
roadio/
├── roadio_firmware/          # ← Your app is here
│   ├── .west/                # West configuration
│   ├── zephyr/               # Zephyr RTOS (v4.3.0)
│   ├── modules/              # Zephyr modules
│   ├── bootloader/           # MCUboot (optional)
│   ├── CMakeLists.txt        # Application build config
│   ├── prj.conf              # Kconfig configuration
│   ├── boards/
│   │   └── nucleo_h723zg.overlay  # Hardware configuration
│   └── src/
│       └── main.c            # Application code
└── CLAUDE.md
```

### Update dependencies (if needed)

If you just cloned the repo or want to update Zephyr modules:

```bash
cd roadio/roadio_firmware
west update
```

---

## Hardware Wiring

**CRITICAL:** Before building, verify your hardware connections match the devicetree overlay!

### CAN (FDCAN1 with SN65HVD230 transceiver)

**Default pins (verify in `boards/nucleo_h723zg.overlay`):**
- **TX:** PD1 (Morpho connector)
- **RX:** PD0 (Morpho connector)

**SN65HVD230 connections:**
```
STM32 PD1 (TX) ──> SN65HVD230 pin 1 (D)
STM32 PD0 (RX) ──> SN65HVD230 pin 4 (R)
3.3V           ──> SN65HVD230 pin 3 (Vcc)
GND            ──> SN65HVD230 pin 2 (GND)
CAN-H          ──> SN65HVD230 pin 7 (CANH)
CAN-L          ──> SN65HVD230 pin 6 (CANL)
```

**OBD-II connector:**
- Pin 6 = CAN-H
- Pin 14 = CAN-L
- Pin 4, 5 = Ground
- Pin 16 = 12V power

**⚠️ WARNING:** Your DB9↔OBD2 cable labeled "RS232" may NOT be wired for CAN! Verify pinout before connecting to a vehicle.

### External Flash (W25Q128 via SPI1)

**Default pins (verify in `boards/nucleo_h723zg.overlay`):**
- **SCK:**  PA5 (Arduino D13)
- **MISO:** PA6 (Arduino D12)
- **MOSI:** PA7 (Arduino D11)
- **CS:**   PD14 (Morpho connector - **adjust as needed**)

**W25Q128 module connections:**
```
W25Q128 VCC  ──> 3.3V (NOT 5V!)
W25Q128 GND  ──> GND
W25Q128 DI   ──> PA7 (MOSI)
W25Q128 DO   ──> PA6 (MISO)
W25Q128 CLK  ──> PA5 (SCK)
W25Q128 CS   ──> PD14 (or your chosen CS pin)
```

### Power (LM2596 Buck Converter)

**For bench testing:**
- Input: 12V from power supply or vehicle OBD-II pin 16
- Output: 5V (set using trim pot)
- Connect 5V output to Nucleo's external power input (NOT USB!)

**⚠️ Note:** When powered via USB, the buck converter is not needed. Use it for vehicle/standalone operation.

---

## Building the Firmware

### 1. Navigate to the application directory

```bash
cd roadio/roadio_firmware
```

### 2. Build for Nucleo-H723ZG

```bash
west build -b nucleo_h723zg -p auto
```

**Flags explained:**
- `-b nucleo_h723zg`: Target board
- `-p auto`: Pristine build (clean before building)

### 3. Build output

If successful, you'll see:
```
[XXX/XXX] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       XXXXX B         1 MB     X.XX%
             RAM:       XXXXX B       564 KB     X.XX%
```

Binary files are in: `build/zephyr/`
- `zephyr.elf` - ELF file (for debugging)
- `zephyr.bin` - Raw binary
- `zephyr.hex` - Intel HEX format

---

## Flashing the Firmware

### Option 1: Using west (recommended)

The Nucleo board has an integrated ST-Link programmer:

```bash
west flash
```

This will automatically:
1. Detect the ST-Link
2. Flash the firmware
3. Reset the board

### Option 2: Using STM32CubeProgrammer (GUI)

1. Download STM32CubeProgrammer from ST's website
2. Connect Nucleo via USB (ST-Link)
3. Open `build/zephyr/zephyr.hex`
4. Click "Connect" then "Download"

### Option 3: Manual (st-flash)

```bash
st-flash write build/zephyr/zephyr.bin 0x8000000
```

---

## Viewing Logs / Console Output

The firmware uses UART for logging (via ST-Link USB serial).

### 1. Find the serial port

**Linux:**
```bash
ls /dev/ttyACM*
# Usually /dev/ttyACM0
```

**macOS:**
```bash
ls /dev/tty.usb*
# Usually /dev/tty.usbmodem*
```

**Windows:**
```
Device Manager → Ports (COM & LPT)
# Usually COM3 or higher
```

### 2. Connect with a serial terminal

**Using screen:**
```bash
screen /dev/ttyACM0 115200
```

**Using minicom:**
```bash
minicom -D /dev/ttyACM0 -b 115200
```

**Using PuTTY (Windows):**
- Serial line: COM3 (or your port)
- Speed: 115200
- Connection type: Serial

### 3. Expected output

```
*** Booting Zephyr OS build v4.3.0 ***
==========================================
Roadio Firmware Starting...
Board: Nucleo-H723ZG
Zephyr Version: 4.3.0
==========================================
[00:00:00.123,000] <inf> roadio: Initializing CAN on can@4000a400
[00:00:00.145,000] <inf> roadio: CAN initialized successfully
[00:00:00.156,000] <inf> roadio: External flash initialized: w25q128jv@0
[00:00:00.167,000] <inf> roadio: Flash size: 16777216 bytes
[00:00:00.178,000] <inf> roadio: Entering main loop...
```

---

## Customizing Hardware Configuration

### Changing CAN pins

Edit `boards/nucleo_h723zg.overlay`:

```dts
&fdcan1 {
    status = "okay";
    pinctrl-0 = <&fdcan1_rx_pa11 &fdcan1_tx_pa12>;  /* Change to PA11/PA12 */
    pinctrl-names = "default";
    /* ... rest of config ... */
};
```

Find available pins in: `zephyr/dts/bindings/pinctrl/st,stm32-pinctrl.yaml`

### Changing SPI CS pin

Edit `boards/nucleo_h723zg.overlay`:

```dts
&spi1 {
    cs-gpios = <&gpiob 6 GPIO_ACTIVE_LOW>;  /* Change CS to PB6 */
    /* ... */
};
```

### Using QSPI instead of SPI

Uncomment the QSPI section in the overlay and wire accordingly.

---

## Debugging

### Enable verbose logging

Edit `prj.conf`:
```conf
CONFIG_LOG_DEFAULT_LEVEL=4  # 4 = DEBUG level
```

Rebuild and flash.

### Use the Zephyr shell

The shell is enabled by default. Connect via serial and type:

```
roadio:~$ help
roadio:~$ can send 0x123 11 22 33 44
roadio:~$ kernel threads
```

### GDB Debugging

```bash
west debug
```

This requires:
- OpenOCD or pyOCD
- GDB (included with Zephyr SDK)

---

## Testing CAN Without a Vehicle

### Enable loopback mode for bench testing

Edit `prj.conf`:
```conf
CONFIG_CAN_LOOPBACK=y
```

In loopback mode, transmitted messages are received by the same device.

Test with shell:
```
roadio:~$ can send 0x7DF 02 01 00
```

You should see the RX callback trigger.

---

## Next Steps

1. **Verify hardware wiring** matches the overlay file
2. **Build and flash** the firmware
3. **Check serial console** for boot messages
4. **Test CAN** in loopback mode first, then with your CAN analyzer
5. **Test flash** read/write operations
6. **Add application logic** in `src/main.c`

---

## Troubleshooting

### Build fails: "no such file or directory"

- Make sure you're in `roadio/roadio_firmware/` when running `west build`
- Run `west update` to fetch dependencies

### Flash fails: "ST-Link not found"

- Check USB connection
- Install ST-Link drivers (Windows)
- Try: `west flash --runner openocd`

### CAN bus-off errors

- Check CAN-H/CAN-L wiring
- Verify bus termination (120Ω resistors)
- Check CAN bus speed matches vehicle (500 kbps for most OBD-II)
- Ensure transceiver power is 3.3V

### No serial output

- Verify correct COM port
- Check baud rate (115200)
- Try pressing reset button on Nucleo
- Check USART3 is enabled in overlay (uncomment if needed)

### SPI flash not detected

- Double-check CS pin matches overlay
- Verify 3.3V power (NOT 5V!)
- Check SPI wiring (MOSI/MISO not swapped)
- Try lowering SPI frequency in overlay

---

## Additional Resources

- **Zephyr Docs:** https://docs.zephyrproject.org/
- **STM32H723 Reference Manual:** https://www.st.com/resource/en/reference_manual/rm0468-stm32h723733-stm32h725735-and-stm32h730-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
- **Nucleo-H723ZG User Manual:** https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf
- **W25Q128 Datasheet:** https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf
- **SN65HVD230 Datasheet:** https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf

---

**Happy hacking! 🚗⚡**
