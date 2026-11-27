#!/usr/bin/env python3
"""PlatformIO pre-build script: make SPI1 usable on F401/F411 Black Pill boards.

Background
----------
On the STM32F401/F411 boards used by NanoLink, the stm32duino SPI driver
configures SPI1 correctly but can leave the peripheral disabled
(``SPI_CR1_SPE`` clear) after ``SPI.begin()`` / ``SPI.beginTransaction()``.
``spi_transfer()`` then waits forever for ``TXE`` and the firmware hangs on the
very first nRF24 register access.

The patch below makes the driver self-correcting: before every transfer it
verifies that SPI1 is clocked, has the correct alternate-function pins, and is
enabled in controller mode. If anything is missing it restores a known-good
configuration (mode 0, ~2.6 MHz). This is idempotent and survives framework
package reinstalls, so a clean checkout always produces working firmware.

The script is shared by the aircraft and ground-station projects through
``extra_scripts = pre:../common/spi_fix.py``.
"""

Import("env")

import os

MARKER_XFER = "/* NANOLINK_SPI_XFER_FIX */"
MARKER_INIT = "/* NANOLINK_SPI_INIT_FIX */"
# Earlier revisions of this fix used a single marker; treat it as equivalent.
LEGACY_MARKERS = ("/* NANOLINK_SPI_FIX */",)


def framework_package_dir():
    """Return the framework package directory, with a portable fallback."""
    try:
        path = env.PioPlatform().get_package_dir("framework-arduinoststm32")
        if path and os.path.isdir(path):
            return path
    except Exception:  # pragma: no cover - depends on PlatformIO version
        pass
    return os.path.join(
        os.path.expanduser("~"), ".platformio", "packages", "framework-arduinoststm32"
    )


SPI_COM = os.path.join(
    framework_package_dir(), "libraries", "SPI", "src", "utility", "spi_com.c"
)

XFER_ANCHOR = (
    "  SPI_TypeDef *_SPI = obj->spi;\n"
    "  uint8_t *tx_buf = (uint8_t *)tx_buffer;\n"
)

XFER_PATCH = (
    XFER_ANCHOR
    + f"""
  {MARKER_XFER}
  /* F401/F411 workaround: spi_init() can leave SPI1 disabled, which makes the
   * TXE wait below spin forever. Repair the peripheral before every transfer. */
#if defined(SPI1)
  if (_SPI == (SPI_TypeDef *)0 || _SPI == (SPI_TypeDef *)-1) {{
    _SPI = SPI1;
  }}
  if (_SPI == SPI1) {{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if (((GPIOA->MODER >> 10) & 3u) != 2u) {{
      /* PA5=SCK, PA6=MISO, PA7=MOSI in alternate function mode. */
      GPIOA->MODER = (GPIOA->MODER & ~((3u << 10) | (3u << 12) | (3u << 14)))
                   | ((2u << 10) | (2u << 12) | (2u << 14));
      GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFu << 20) | (0xFu << 24) | (0xFu << 28)))
                    | ((5u << 20) | (5u << 24) | (5u << 28));
    }}
    if (!(SPI1->CR1 & SPI_CR1_SPE) || !(SPI1->CR1 & SPI_CR1_MSTR)) {{
      /* Controller mode, software NSS, mode 0, prescaler /32. */
      SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (5u << 3) | SPI_CR1_SPE;
    }}
  }}
#endif
"""
)

INIT_ANCHOR = (
    "    /* In order to set correctly the SPI polarity we need to enable the peripheral */\n"
    "    LL_SPI_Enable(obj->spi);\n"
)

INIT_PATCH = (
    INIT_ANCHOR
    + f"""
    {MARKER_INIT}
#if defined(SPI1)
    if (obj->spi == SPI1) {{
      SPI1->CR1 |= SPI_CR1_SPE;
    }}
#endif
"""
)


def apply_patch(content, marker, anchor, replacement, label):
    if marker in content or any(m in content for m in LEGACY_MARKERS):
        print(f"[spi_fix] {label}: already applied")
        return content
    if anchor not in content:
        raise SystemExit(
            f"[spi_fix] ERROR: could not find the {label} anchor in\n  {SPI_COM}\n"
            "The stm32duino SPI driver layout changed. Update firmware/common/spi_fix.py."
        )
    print(f"[spi_fix] {label}: patching {SPI_COM}")
    return content.replace(anchor, replacement, 1)


def patch_driver():
    if not os.path.isfile(SPI_COM):
        raise SystemExit(
            f"[spi_fix] ERROR: SPI driver not found at\n  {SPI_COM}\n"
            "Install the stm32duino framework first (pio run)."
        )

    with open(SPI_COM, "r", encoding="utf-8") as handle:
        content = handle.read()

    original = content
    content = apply_patch(content, MARKER_INIT, INIT_ANCHOR, INIT_PATCH, "spi_init")
    content = apply_patch(content, MARKER_XFER, XFER_ANCHOR, XFER_PATCH, "spi_transfer")

    if content != original:
        with open(SPI_COM, "w", encoding="utf-8") as handle:
            handle.write(content)

    with open(SPI_COM, "r", encoding="utf-8") as handle:
        verify = handle.read()
    transfer_ok = MARKER_XFER in verify or any(m in verify for m in LEGACY_MARKERS)
    if not transfer_ok or MARKER_INIT not in verify:
        raise SystemExit("[spi_fix] ERROR: patch verification failed")


# Pre-scripts run while the SCons environment is being built, so apply the
# patch immediately instead of registering a callback.
patch_driver()
