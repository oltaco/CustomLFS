# CustomLFS Library

This library supports both nRF5x on-chip flash (including custom areas not just the 28kb UserData area!) and external SPI flash memory chips, providing a unified filesystem interface for embedded applications.


## Installation

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps = 
    https://github.com/oltaco/CustomLFS.git
```

## Quick Start

### nRF5x Internal Flash
Probably requires the Adafruit nRF52 Arduino framework?
```cpp
#include "CustomLFS.h"

CustomLFS myfs;

void setup() {
  if (!myfs.begin()) {
    Serial.println("nRF5x flash init failed");
    return;
  }
  
  // Use filesystem
  File file = myfs.open("/config.txt", FILE_WRITE);
  file.println("Hello nRF5x!");
  file.close();
}
```

### SPI Flash Memory
Might not require any other dependencies? I haven't checked.
```cpp
#include "CustomLFS_SPIFlash.h"

const int chipSelect = 25;

void setup() {
  // Option 1: Use global instance
  if (!FlashFS.begin(chipSelect, SPI)) {
    Serial.println("SPI flash init failed");
    return;
  }
  
  Serial.print("Detected: ");
  Serial.println(FlashFS.getChipName());
  
  // Option 2: Create custom instance
  CustomLFS_SPIFlash myFlash(chipSelect, SPI);
  if (!myFlash.begin()) {
    Serial.println("Custom instance init failed");
    return;
  }
}
```

## Advanced Usage

### Custom nRF5x Flash Region

```cpp
#include "CustomLFS.h"

CustomLFS myfs;

void setup() {
  // Configure custom flash region (address, size, block_size)
// This example would use 100kb at the of the application flash area.
  if (!myfs.setFlashRegion(0xD4000, 0x19000, 128)) {
    Serial.println("Invalid flash region");
    return;
  }
  
  if (!myfs.begin()) {
    Serial.println("Custom region init failed");
    return;
  }
}
```


## License

This library is released under the MIT License. See `LICENSE` file for details.

## Acknowledgments


- Based on [Paul Stoffregen's LittleFS](https://github.com/PaulStoffregen/LittleFS) implementation
