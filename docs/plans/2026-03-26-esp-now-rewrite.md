# ESP-NOW Rewrite Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the BLE GAP transport with ESP-NOW v2 broadcast while keeping the existing `SimpleRadio` API close to current usage and adding blob messaging.

**Architecture:** Split the work into two layers. First, introduce a transport-agnostic packet codec that owns the on-wire format, data typing, and duplicate detection keys. Then rewrite `SimpleRadioImpl` to initialize Wi-Fi + ESP-NOW, broadcast encoded packets, and decode incoming frames into the existing callbacks plus a new blob callback.

**Tech Stack:** ESP-IDF / Arduino-compatible C++, ESP-NOW (`esp_now.h`), Wi-Fi (`esp_wifi.h`), FreeRTOS, small native codec test compiled with `g++`.

---

### Task 1: Add a codec seam and failing tests

**Files:**
- Create: `tests/codec_test.cpp`
- Create: `src/simple_radio_codec.h`
- Create: `src/simple_radio_codec.cpp`

**Step 1: Write the failing test**

Add tests that encode and decode:
- string packets with 1-byte group
- number packets with 8-byte double payload
- key/value packets with `double + key bytes`
- blob packets with arbitrary bytes
- invalid packets rejected cleanly

**Step 2: Run test to verify it fails**

Run: `g++ -std=gnu++14 -Isrc tests/codec_test.cpp src/simple_radio_codec.cpp -o /tmp/simple_radio_codec_test`
Expected: FAIL because codec files or symbols do not exist yet.

**Step 3: Write minimal implementation**

Implement codec helpers that:
- define the new packet header
- encode outgoing payloads into a buffer sized for ESP-NOW v2 frames
- decode incoming packets into typed views

**Step 4: Run test to verify it passes**

Run: `g++ -std=gnu++14 -Isrc tests/codec_test.cpp src/simple_radio_codec.cpp -o /tmp/simple_radio_codec_test && /tmp/simple_radio_codec_test`
Expected: PASS with exit code 0.

### Task 2: Rewrite the runtime transport to ESP-NOW

**Files:**
- Modify: `src/simple_radio.h`
- Modify: `src/simple_radio.cpp`
- Modify: `CMakeLists.txt`

**Step 1: Write the failing test**

Add/extend codec tests for any API-visible format details needed by the runtime:
- duplicate detection key stability
- max payload limits
- full 6-byte address handling

**Step 2: Run test to verify it fails**

Run the codec test binary again.
Expected: FAIL on the new expectations.

**Step 3: Write minimal implementation**

Update runtime code to:
- replace BLE includes/config with Wi-Fi + ESP-NOW
- change group range from `<0,16)` to `<0,256)`
- use full MAC address in `PacketInfo`
- preserve `sendString`, `sendNumber`, `sendKeyValue`, `setData`
- add `PacketDataType::Blob`, `sendBlob`, and blob callback registration
- keep broadcast-only sending
- use ESP-NOW v2 extended payload sizing

**Step 4: Run test to verify it passes**

Run the codec test binary and an ESP-IDF example build.
Expected: codec test passes and the project compiles.

### Task 3: Update examples and documentation

**Files:**
- Modify: `README.md`
- Modify: `examples/basic/main.cpp`
- Modify: `library.json`

**Step 1: Write the failing test**

Use the example build as the acceptance check after docs/API updates.

**Step 2: Run test to verify it fails**

Run: `pio run -e esp32dev -c test-inis/esp32-idf5-idf.ini`
Expected: If the example/docs drift from the API, build fails.

**Step 3: Write minimal implementation**

Document:
- ESP-NOW transport instead of BLE
- larger payload support
- blob API
- any Wi-Fi/channel constraints that materially affect usage

Update the example to exercise blob sending or receiving.

**Step 4: Run test to verify it passes**

Run the same PlatformIO build command.
Expected: PASS with exit code 0.

### Task 4: Final verification

**Files:**
- Verify only

**Step 1: Run full verification**

Run:
- `g++ -std=gnu++14 -Isrc tests/codec_test.cpp src/simple_radio_codec.cpp -o /tmp/simple_radio_codec_test && /tmp/simple_radio_codec_test`
- `pio run -e esp32dev -c test-inis/esp32-idf5-idf.ini`

**Step 2: Inspect results**

Confirm:
- codec tests pass
- library/example compile cleanly
- public docs match the rewritten API
