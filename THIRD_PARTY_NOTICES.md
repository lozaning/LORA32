# Third-party sources and notices

The firmware source is distributed under **GNU GPL version 3 or later**. The
full license is in [LICENSE](LICENSE). Retain original notices when redistributing
this tree or adaptations. Included prebuilt application code also incorporates
ESP-IDF components with their own terms. This document records provenance; it
does not replace the individual license texts and source headers.

## ESPARGOS SDR work

The project builds on [ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr/tree/550fadea4d00a9e26ce921c5832167becb3dc20c),
commit `550fadea4d00a9e26ce921c5832167becb3dc20c`, GPL-3.0-or-later.
Continuous ADC acquisition follows its measured bank/boundary strategy.
The unaligned PIE load sequence in `main/targets/esp32s3/modem_vector.S` adapts
upstream `s3_unpack.S`; its GPL attribution is retained. Original receiver,
display and driver source notices are retained in the exported snapshot.

## LoRa coding reference

The C SF7 header, Hamming, diagonal-interleaver and Gray-mapping port follows
[tapparelj/gr-lora_sdr](https://github.com/tapparelj/gr-lora_sdr/tree/862746dd1cf635c9c8a4bfbaa2c3a0ec3a5306c9),
commit `862746dd1cf635c9c8a4bfbaa2c3a0ec3a5306c9`, GNU GPL v3.
The adapted `lora_codec.c` is marked GPL-3.0-or-later. The streaming receiver,
wire2 transport and terminal wrapper are implemented in this project.
The small host known-answer test uses the same saved gr-lora deadbeef symbol
vector as the firmware LORAKAT diagnostic.

Internal Python reference ports and third-party RF recordings used during
research are not bundled in this export. Their inclusion would require
preserving their separate provenance and redistribution terms. The validation
summary describes their testing role without redistributing those files.

## FFT and ESP-DSP

`main/targets/esp32s3/modem_fft.S` adapts ESPARGOS `s3_fft_rnd.S`, originally
Espressif ESP-DSP `dsps_fft2r_sc16_aes3_`. It retains **Copyright 2018–2019
Espressif Systems (Shanghai) PTE LTD**, Apache License 2.0. Naming and IRAM
placement were changed for this streaming decoder. See
[licenses/ESP-DSP-Apache-2.0.txt](licenses/ESP-DSP-Apache-2.0.txt).
The research reference revision of
[ESP-DSP](https://github.com/espressif/esp-dsp/tree/a53a0756833c045311ea1d79a2badf495cdfde4c)
is `a53a0756833c045311ea1d79a2badf495cdfde4c`; no entire ESP-DSP package is
vendored or required by this build.

## ESP-IDF and precompiled radio libraries

The build requires [ESP-IDF](https://github.com/espressif/esp-idf/tree/25fe69f946311abdaf9ad56591f25fedbc20ac98)
commit `25fe69f946311abdaf9ad56591f25fedbc20ac98`. See
[licenses/ESP-IDF-Apache-2.0.txt](licenses/ESP-IDF-Apache-2.0.txt).
The SDK is obtained separately with its pinned submodules; it is not inside
this upload folder. Some PHY/Wi-Fi behavior resides in Espressif-supplied
precompiled libraries. This export does not claim to contain their source.

| SDK radio submodule | Pinned revision | Included notice |
|---|---|---|
| `components/esp_phy/lib` | `e294ff039e26b3486d6c9e5853d24d98ee3300b2` | [Espressif-PHY.txt](licenses/Espressif-PHY.txt) |
| `components/esp_wifi/lib` | `77143eec456e1cdcbce3e1dad8d5983cd28a368f` | [Espressif-WiFi.txt](licenses/Espressif-WiFi.txt) |
| `components/esp_coex/lib` | `3c490f8059e0a7eeedda30aaa1e4b3afad06f632` | [Espressif-Coex.txt](licenses/Espressif-Coex.txt) |

The library SHA256 values are in [docs/provenance.json](docs/provenance.json).
Precompiled `.a` archives and certification firmware from other research
downloads are not redistributed as standalone components here.

ESP-IDF includes components with separate notices, retained here as applicable
upstream license texts:

- [lwIP](licenses/lwIP.txt)
- [Mbed TLS](licenses/Mbed-TLS.txt)
- [TF-PSA-Crypto](licenses/TF-PSA-Crypto.txt)
- [WPA Supplicant](licenses/WPA-Supplicant.txt)

See the pinned SDK for complete component-level notices and its dependency tree.
The unused LCD source files retain their own author/license headers even though
the INFO19 build uses `display_modem.c` instead.

## Host dependencies

The optional host tools install, rather than vendor,
[pyserial](https://github.com/pyserial/pyserial) (BSD license) and
[esptool](https://github.com/espressif/esptool) (GPL-2.0-or-later).
Their distribution packages contain their own license texts.
The clock helper also installs [tzdata](https://github.com/python/tzdata)
(Apache-2.0, with the database's public-domain notice).
