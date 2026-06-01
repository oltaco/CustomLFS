/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 hathach for Adafruit Industries
 * Copyright (c) 2026 oltaco <taco@sly.nu>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "CustomLFS2.h"

//--------------------------------------------------------------------+
// LFS Disk IO Callbacks
//--------------------------------------------------------------------+

int CustomLFS2::_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
  CustomLFS2* fs = (CustomLFS2*)c->context;
  uint32_t addr = fs->lba2addr(block) + off;
  // disallow out of bounds
  if (addr < fs->_flash_addr || (addr + size) > (fs->_flash_addr + fs->_flash_total_size)) {
      return LFS_ERR_IO;
  }

  // if block size == page size we can bypass the cache
  if (c->block_size >= FLASH_NRF52_PAGE_SIZE) {
    // Serial.printf("LFS read direct: block=%u off=%u size=%u addr=0x%08X\n", block, off, size, addr);
    memcpy(buffer, (const void *) addr, size);
    return 0;
  }

  // Serial.printf("LFS read cache: block=%u off=%u size=%u addr=0x%08X\n", block, off, size, addr);
  VERIFY(flash_nrf5x_read(buffer, addr, size) > 0, -1);
  return 0;
}

int CustomLFS2::_flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {     
  CustomLFS2* fs = (CustomLFS2*)c->context;
  uint32_t addr = fs->lba2addr(block) + off;
  const uint8_t *src = (const uint8_t *)buffer;
  // disallow out of bounds
  if (addr < fs->_flash_addr || (addr + size) > (fs->_flash_addr + fs->_flash_total_size)) {
      return LFS_ERR_IO;
  }

  if (c->block_size >= FLASH_NRF52_PAGE_SIZE) {
    // Serial.printf("LFS prog direct: block=%u off=%u size=%u addr=0x%08X\n", block, off, size, addr);
    uint32_t remaining = size;

    #if defined(NRF52840_XXAA)
    // workaround for nRF52840 S140 v6.1.1 bug: full page writes cause an assert
    const uint32_t max_chunk = FLASH_NRF52_PAGE_SIZE / 2;
    #else
    const uint32_t max_chunk = FLASH_NRF52_PAGE_SIZE;
    #endif

    while (remaining) {
        uint32_t chunk = (remaining > max_chunk) ? max_chunk : remaining;
        if (!fal_sub_program(addr, src, chunk)) return LFS_ERR_IO;
        addr += chunk;
        src += chunk;
        remaining -= chunk;
    }
    return 0;
  }
  
  // Serial.printf("LFS prog cache: block=%u off=%u size=%u addr=0x%08X\n", block, off, size, addr);
  VERIFY(flash_nrf5x_write(addr, buffer, size), -1);

  return 0;
}

int CustomLFS2::_flash_erase(const struct lfs_config *c, lfs_block_t block)
{
  CustomLFS2* fs = (CustomLFS2*)c->context;
  uint32_t addr = fs->lba2addr(block);

  // if block size == page size we can bypass the cache and do a direct erase
  if (c->block_size >= FLASH_NRF52_PAGE_SIZE) {
      // Serial.printf("LFS erase direct: block=%u addr=0x%08X\n", block, addr);
      return fal_erase(addr) ? 0 : LFS_ERR_IO;
  }

  // Implement as write 0xff to whole block address
  // Serial.printf("LFS erase cache: block=%u addr=0x%08X\n", block, addr);
  for(int i = 0; i < fs->_block_size; i++)
  {
    flash_nrf5x_write8(addr + i, 0xFF);
  }
  flash_nrf5x_flush();

  return 0;
}

int CustomLFS2::_flash_sync(const struct lfs_config *c)
{
  (void) c;
  // only flush if using the cache
  if (c->block_size < FLASH_NRF52_PAGE_SIZE) {
      flash_nrf5x_flush();
  }

  return 0;
}

//--------------------------------------------------------------------+
// CustomLFS2 Implementation
//--------------------------------------------------------------------+

CustomLFS2::CustomLFS2(void)
  : Adafruit_LittleFS(&_lfs_config)
  , _flash_addr(LFS2_DEFAULT_FLASH_ADDR)
  , _flash_total_size(LFS2_DEFAULT_FLASH_TOTAL_SIZE)
  , _block_size(LFS2_DEFAULT_BLOCK_SIZE)
{
  _configure_lfs();
}

CustomLFS2::CustomLFS2(uint32_t flash_addr, uint32_t flash_size, uint32_t block_size)
  : Adafruit_LittleFS(&_lfs_config)
  , _flash_addr(flash_addr)
  , _flash_total_size(flash_size)
  , _block_size(block_size)
{
  // Validate the configuration
  if (!validateFlashRegion(flash_addr, flash_size, block_size)) {
    // Fall back to default configuration if invalid
    _flash_addr = LFS2_DEFAULT_FLASH_ADDR;
    _flash_total_size = LFS2_DEFAULT_FLASH_TOTAL_SIZE;
    _block_size = LFS2_DEFAULT_BLOCK_SIZE;
  }
  
  _configure_lfs();
}

CustomLFS2::CustomLFS2(bool auto_configure)
  : Adafruit_LittleFS(&_lfs_config)
  , _flash_addr(0)
  , _flash_total_size(0)
  , _block_size(0)
{
  // Clear the config but don't configure if auto_configure is false
  memset(&_lfs_config, 0, sizeof(_lfs_config));
  if (auto_configure) {
    _flash_addr = LFS2_DEFAULT_FLASH_ADDR;
    _flash_total_size = LFS2_DEFAULT_FLASH_TOTAL_SIZE;
    _block_size = LFS2_DEFAULT_BLOCK_SIZE;
    _configure_lfs();
  }
}

void CustomLFS2::_configure_lfs(const lfs_config* override) {
  // Clear the configuration structure
  memset(&_lfs_config, 0, sizeof(_lfs_config));
  
  // Set up the configuration
  _lfs_config.context = this;
  
  // Block device operations
  _lfs_config.read = _flash_read;
  _lfs_config.prog = _flash_prog;
  _lfs_config.erase = _flash_erase;
  _lfs_config.sync = _flash_sync;
  
  // Block device configuration
  // NOTE: read and prog size used to be set to _block_size, but they are set to 128 because 4096 would be too large.
  // TODO: ?consider making these configurable for performance tuning

  _lfs_config.block_size = _block_size;
  _lfs_config.block_count = _flash_total_size / _block_size;

  // Buffers (set to NULL for dynamic allocation)
  _lfs_config.read_buffer = NULL;
  _lfs_config.prog_buffer = NULL;
  _lfs_config.lookahead_buffer = NULL;

  if (override) {
    _lfs_config.read_size = override->read_size;
    _lfs_config.prog_size = override->prog_size;
    _lfs_config.cache_size = override->cache_size;
    _lfs_config.lookahead_size = override->lookahead_size;
    _lfs_config.block_cycles = override->block_cycles;
  } else {
    _lfs_config.read_size = 64;         // must be >= prog_size && <= block_size
    _lfs_config.prog_size = 64;         // keeps metadata appends small, to reduce compaction overhead
    _lfs_config.lookahead_size = 32;    // lookahead buffer in bytes (32*8 = 256 blocks)
    _lfs_config.block_cycles = 512;     // wear levelling interval
    _lfs_config.cache_size = min((uint32_t)2048, _block_size); // 2048 is the largest we can write on nRF52840
  }
  
}

bool CustomLFS2::setFlashRegion(uint32_t flash_addr, uint32_t flash_size, uint32_t block_size)
{
  if (_mounted) {
    return false;  // Cannot change configuration if already mounted
  }
  
  // Validate the configuration
  if (!validateFlashRegion(flash_addr, flash_size, block_size)) {
    return false;
  }
  
  // Update configuration
  _flash_addr = flash_addr;
  _flash_total_size = flash_size;
  _block_size = block_size;
  
  // Reconfigure LFS
  _configure_lfs();
  
  return true;
}

bool CustomLFS2::setFlashRegion(uint32_t flash_addr, uint32_t flash_size, uint32_t block_size, const lfs_config& config) {
    if (_mounted) return false;
    if (!validateFlashRegion(flash_addr, flash_size, block_size)) return false;
    _flash_addr = flash_addr;
    _flash_total_size = flash_size;
    _block_size = block_size;
    _configure_lfs(&config);
    return true;
}

bool CustomLFS2::validateFlashRegion(uint32_t flash_addr, uint32_t flash_size, uint32_t block_size)
{
  // Basic validation
  if (flash_addr == 0 || flash_size == 0 || block_size == 0) {
    return false;
  }
  
  // Check alignment — flash address should be page-aligned for efficiency
  if (flash_addr % FLASH_NRF52_PAGE_SIZE != 0) {
    return false;
  }
  
  // Block size should be reasonable (at least 16 bytes, at most page size)
  if (block_size < 128 || block_size > FLASH_NRF52_PAGE_SIZE) {
    return false;
  }
  
  // Flash size should be multiple of block size
  if (flash_size % block_size != 0) {
    return false;
  }
  
  // Check if region is within valid flash range
  #ifdef NRF52840_XXAA
    uint32_t max_flash = 0x100000;  // 1MB
  #else
    uint32_t max_flash = 0x80000;   // 512KB
  #endif
  
  if (flash_addr >= max_flash || (flash_addr + flash_size) > max_flash) {
    return false;
  }
  
  return true;
}

bool CustomLFS2::begin(bool autoFormat) {
  // try to mount
  if (Adafruit_LittleFS::begin(&_lfs_config)) {
      return true;
  }

  // mount failed, try to migrate from LFS1
  int err = lfs_migrate(&_lfs, &_lfs_config);
  if (err == LFS_ERR_OK) {
      // migration successful, try to mount again
      if (Adafruit_LittleFS::begin(&_lfs_config)) {
          return true;
      }
  }

  if (!autoFormat) {
    return false; // mount failed and autoFormat is disabled, give up
  }

  // Auto-format: erase all sectors in our flash region and format
  for (uint32_t addr = _flash_addr; addr < _flash_addr + _flash_total_size; addr += FLASH_NRF52_PAGE_SIZE) {
    VERIFY(flash_nrf5x_erase(addr));
  }

  // Format the filesystem
  this->format();

  // Try to mount again — if this fails, give up
  if (!Adafruit_LittleFS::begin()) {
    return false;
  }

  return true;
}

int CustomLFS2::migrate(void)
{
  // Migrate a LittleFS v1 filesystem to v2 in-place.
  // The _lfs_config must be set up with the correct flash callbacks
  // and block size matching the existing v1 filesystem.
  // Returns LFS_ERR_OK (0) on success, negative LFS error code on failure.
  return lfs_migrate(&_lfs, &_lfs_config);
}

bool CustomLFS2::formatRegion(void)
{
  if (_mounted) {
    end();
  }
  
  // Erase all sectors in our flash region
  for (uint32_t addr = _flash_addr; addr < _flash_addr + _flash_total_size; addr += FLASH_NRF52_PAGE_SIZE) {
    if (!flash_nrf5x_erase(addr)) {
      return false;
    }
  }
  
  // Format the filesystem
  return format();
}