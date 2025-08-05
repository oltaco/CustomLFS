/* 
 * CustomLFS_QSPIFlash.cpp - QSPI Flash support for CustomLFS
 * Version v0.1
 * 
 * Copyright (c) 2025 oltaco <taco@sly.nu>
 *
 * The MIT License (MIT)
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

#ifdef NRF52840_XXAA

#include "CustomLFS_QSPIFlash.h"
#include "Adafruit_TinyUSB.h"

// Check if QSPI is enabled
#if !defined(NRFX_QSPI_ENABLED) || (NRFX_QSPI_ENABLED != 1)
#error "QSPI must be enabled. Add -DNRFX_QSPI_ENABLED=1 to build flags"
#endif

// Simple QSPI Flash chip database - minimal for compatibility
const QSPIFlashChip qspiFlashChips[] = {
  // Winbond W25Q16JV
  {
    .jedec_id = {0xEF, 0x40, 0x15},
    .total_size = 2097152,              // 2MB
    .sector_size = 4096,
    .page_size = 256,
    .address_bits = 24,
    .read_opcode = 0xEB,
    .program_opcode = 0x32,
    .erase_opcode = 0x20,
    .status_opcode = 0x05,
    .supports_quad_read = true,
    .supports_quad_write = true,
    .quad_enable_register = 2,
    .quad_enable_bit = 1,
    .quad_enable_volatile = false,
    .max_clock_hz = 80000000,
    .write_timeout_ms = 5,
    .erase_timeout_ms = 400,
    .startup_delay_us = 10000,
    .name = "W25Q16JV"
  },
  // PUYA P25Q16H - WioTrackerL1 / Xiao NRF52840
  {
    .jedec_id = {0x85, 0x60, 0x15},
    .total_size = 2097152,
    .sector_size = 4096,
    .page_size = 256,
    .address_bits = 24,
    .read_opcode = 0xEB,
    .program_opcode = 0x32,
    .erase_opcode = 0x20,
    .status_opcode = 0x05,
    .supports_quad_read = true,
    .supports_quad_write = true,
    .quad_enable_register = 2,
    .quad_enable_bit = 1,
    .quad_enable_volatile = false,
    .max_clock_hz = 55000000,
    .write_timeout_ms = 5,
    .erase_timeout_ms = 300,
    .startup_delay_us = 10000,
    .name = "P25Q16H"
  },
  // Macronix MX25R1635F (Thinknode M1)
  {
    .jedec_id = {0xC2, 0x28, 0x15},
    .total_size = 2097152,              // 2MB
    .sector_size = 4096,
    .page_size = 256,
    .address_bits = 24,
    .read_opcode = 0xEB,
    .program_opcode = 0x32,
    .erase_opcode = 0x20,
    .status_opcode = 0x05,
    .supports_quad_read = true,
    .supports_quad_write = true,
    .quad_enable_register = 2,
    .quad_enable_bit = 1,
    .quad_enable_volatile = false,
    .max_clock_hz = 80000000,           // 8MHz max for quad mode in low power mode
    .write_timeout_ms = 5,
    .erase_timeout_ms = 400,
    .startup_delay_us = 10000,
    .name = "MX25R1635F"
  }
};


const uint32_t qspiFlashChipCount = sizeof(qspiFlashChips) / sizeof(qspiFlashChips[0]);

// Global instance
CustomLFS_QSPIFlash QSPIFlash;

//--------------------------------------------------------------------+
// Event Handler - Simplified for compatibility
//--------------------------------------------------------------------+

void CustomLFS_QSPIFlash::qspi_event_handler(nrfx_qspi_evt_t event, void *p_context)
{
  CustomLFS_QSPIFlash* fs = (CustomLFS_QSPIFlash*)p_context;
  
  if (fs) {
    fs->_operation_pending = false;  // Always clear the pending flag
    fs->_last_error = NRFX_SUCCESS;  // Assume success (could be enhanced)
  }
}

//--------------------------------------------------------------------+
// LittleFS Callbacks
//--------------------------------------------------------------------+

int CustomLFS_QSPIFlash::_qspi_read(const struct lfs_config *c, lfs_block_t block, 
                               lfs_off_t off, void *buffer, lfs_size_t size)
{
  CustomLFS_QSPIFlash* fs = (CustomLFS_QSPIFlash*)c->context;
  uint32_t addr = block * c->block_size + off;
  
  return fs->qspiRead(addr, buffer, size) ? 0 : -1;
}

int CustomLFS_QSPIFlash::_qspi_prog(const struct lfs_config *c, lfs_block_t block, 
                               lfs_off_t off, const void *buffer, lfs_size_t size)
{
  CustomLFS_QSPIFlash* fs = (CustomLFS_QSPIFlash*)c->context;
  uint32_t addr = block * c->block_size + off;
  
  return fs->qspiWrite(addr, buffer, size) ? 0 : -1;
}

int CustomLFS_QSPIFlash::_qspi_erase(const struct lfs_config *c, lfs_block_t block)
{
  CustomLFS_QSPIFlash* fs = (CustomLFS_QSPIFlash*)c->context;
  uint32_t addr = block * c->block_size;
  
  return fs->qspiErase(addr) ? 0 : -1;
}

int CustomLFS_QSPIFlash::_qspi_sync(const struct lfs_config *c)
{
  CustomLFS_QSPIFlash* fs = (CustomLFS_QSPIFlash*)c->context;
  return fs->qspiWaitReady(5000) ? 0 : -1;  // 5 second timeout
}

//--------------------------------------------------------------------+
// CustomLFS_QSPIFlash Implementation
//--------------------------------------------------------------------+

CustomLFS_QSPIFlash::CustomLFS_QSPIFlash()
  : CustomLFS(false)  // Don't auto-configure
  , _qspi_initialized(false)
  , _quad_mode_enabled(false)
  , _chip(nullptr)
  , _total_size(2097152)    // Default 1MB
  , _sector_size(4096)      // Default 4KB sectors
  , _page_size(256)         // Default 256B pages
  , _is_4byte_addr(false)
  , _use_quad_read(false)
  , _use_quad_write(false)
  , _clock_frequency(16000000)  // Default 16MHz
  , _operation_pending(false)
  , _last_error(NRFX_SUCCESS)
{
  memset(&_qspi_config, 0, sizeof(_qspi_config));
}

CustomLFS_QSPIFlash::~CustomLFS_QSPIFlash()
{
  if (_qspi_initialized) {
    nrfx_qspi_uninit();
    _qspi_initialized = false;
  }
}

bool CustomLFS_QSPIFlash::begin(uint8_t sck_pin, uint8_t csn_pin, uint8_t io0_pin, 
                           uint8_t io1_pin, uint8_t io2_pin, uint8_t io3_pin)
{
  if (_qspi_initialized) {
    return false;
  }
  
  Serial.println("Starting QSPI initialization...");
  
  // Configure using the working example's approach
  memset(&_qspi_config, 0, sizeof(_qspi_config));
  
  // Pin configuration
  _qspi_config.pins.sck_pin = g_ADigitalPinMap[sck_pin];
  _qspi_config.pins.csn_pin = g_ADigitalPinMap[csn_pin];
  _qspi_config.pins.io0_pin = g_ADigitalPinMap[io0_pin];
  _qspi_config.pins.io1_pin = g_ADigitalPinMap[io1_pin];
  _qspi_config.pins.io2_pin = g_ADigitalPinMap[io2_pin];
  _qspi_config.pins.io3_pin = g_ADigitalPinMap[io3_pin];
  
  // Protocol configuration (start with basics) see qspi hal
  _qspi_config.prot_if.readoc = NRF_QSPI_READOC_FASTREAD;  // single data line
  _qspi_config.prot_if.writeoc = NRF_QSPI_WRITEOC_PP;      // single data line  
  _qspi_config.prot_if.addrmode = NRF_QSPI_ADDRMODE_24BIT;
  _qspi_config.prot_if.dpmconfig = false;  // Disable DPM initially
  
  // Physical interface
  _qspi_config.phy_if.sck_freq = NRF_QSPI_FREQ_32MDIV2;    // 16MHz
  _qspi_config.phy_if.spi_mode = NRF_QSPI_MODE_0;
  _qspi_config.phy_if.dpmen = false;
  
  Serial.print("QSPI pins: SCK=");
  Serial.print(sck_pin);
  Serial.print(" CSN=");
  Serial.print(csn_pin);
  Serial.print(" IO0=");
  Serial.print(io0_pin);
  Serial.print(" IO1=");
  Serial.print(io1_pin);
  Serial.print(" IO2=");
  Serial.print(io2_pin);
  Serial.print(" IO3=");
  Serial.println(io3_pin);
  
  // Initialize QSPI using blocking mode
  nrfx_err_t err = nrfx_qspi_init(&_qspi_config, NULL, NULL);
  if (err != NRFX_SUCCESS) {
    Serial.print("QSPI init failed: 0x");
    Serial.println(err, HEX);
    return false;
  }
  
  Serial.println("QSPI driver initialized");
  
  
  // Activate QSPI tasks (this was missing!)
  Serial.println("Activating QSPI...");
  NRF_QSPI->TASKS_ACTIVATE = 1;
  
  // Wait for QSPI to be ready
  Serial.println("Waiting for QSPI ready...");
  if (!waitForQSPIReady()) {
    Serial.println("QSPI ready timeout!");
    return false;
  }
  
  Serial.println("QSPI is ready");
  _qspi_initialized = true;
  
  // testFlash();
  // testLFSCallbacks();

  // Continue with filesystem setup...
  _configure_lfs();
  
  if (!Adafruit_LittleFS::begin(&_lfs_config)) {
    Serial.println("Mount failed, formatting...");
    if (!format()) {
      Serial.println("Format failed!");
      return false;
    }
    if (!Adafruit_LittleFS::begin(&_lfs_config)) {
      Serial.println("Mount failed after format!");
      return false;
    }
    Serial.println("Formatted and mounted");
  } else {
    Serial.println("Filesystem mounted");
  }
  
  return true;
}

bool CustomLFS_QSPIFlash::testLFSCallbacks()
{
  Serial.println("Testing LittleFS callbacks directly...");
  
  uint8_t testBuffer[256];
  memset(testBuffer, 0xAA, sizeof(testBuffer));
  
  // Test erase
  Serial.println("Testing erase callback...");
  int result = _qspi_erase(&_lfs_config, 0);  // Erase block 0
  if (result != 0) {
    Serial.print("Erase callback failed: ");
    Serial.println(result);
    return false;
  }
  
  // Test write
  Serial.println("Testing write callback...");
  result = _qspi_prog(&_lfs_config, 0, 0, testBuffer, sizeof(testBuffer));
  if (result != 0) {
    Serial.print("Write callback failed: ");
    Serial.println(result);
    return false;
  }
  
  // Test read
  Serial.println("Testing read callback...");
  uint8_t readBuffer[256];
  result = _qspi_read(&_lfs_config, 0, 0, readBuffer, sizeof(readBuffer));
  if (result != 0) {
    Serial.print("Read callback failed: ");
    Serial.println(result);
    return false;
  }
  
  // Verify data
  bool dataOk = true;
  for (int i = 0; i < 256; i++) {
    if (readBuffer[i] != 0xAA) {
      dataOk = false;
      break;
    }
  }
  
  Serial.print("Callback test: ");
  Serial.println(dataOk ? "SUCCESS" : "FAILED");
  return dataOk;
}

bool CustomLFS_QSPIFlash::configureP25Q16H()
{
  // Reset enable command
  nrf_qspi_cinstr_conf_t cinstr_cfg = {
    .opcode = 0x66,  // RSTEN
    .length = NRF_QSPI_CINSTR_LEN_1B,
    .io2_level = true,
    .io3_level = true,
    .wipwait = false,
    .wren = true
  };
  
  if (nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL) != NRFX_SUCCESS) {
    Serial.println("Reset enable failed");
    return false;
  }
  
  // Reset command
  cinstr_cfg.opcode = 0x99;  // RST
  if (nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL) != NRFX_SUCCESS) {
    Serial.println("Reset command failed");
    return false;
  }
  
  // Configure for QSPI mode - Write Status Register
  uint8_t status_data[] = {0x00, 0x02};  // Enable QE bit
  cinstr_cfg.opcode = 0x01;  // WRSR
  cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_3B;
  
  if (nrfx_qspi_cinstr_xfer(&cinstr_cfg, status_data, NULL) != NRFX_SUCCESS) {
    Serial.println("QSPI mode enable failed");
    return false;
  } else {
    Serial.println("QSPI quad mode enabled");
  }
  
  Serial.println("P25Q16H configured for QSPI mode");
  return true;
}

bool CustomLFS_QSPIFlash::waitForQSPIReady()
{
  uint32_t timeout = millis() + 5000;
  
  while (millis() < timeout) {
    // Use the same check as the working example
    uint32_t status = NRF_QSPI->STATUS;
    if ((status & 0x08) == 0x08 && (status & 0x01000000) == 0) {
      return true;
    }
    delay(1);
  }
  
  return false;
}

bool CustomLFS_QSPIFlash::detectChip()
{
  uint8_t jedec_id[3] = {0, 0, 0};
  
  // Try to read JEDEC ID
  nrf_qspi_cinstr_conf_t cinstr_cfg = {
    .opcode = 0x9F,
    .length = NRF_QSPI_CINSTR_LEN_4B,
    .io2_level = true,
    .io3_level = true,
    .wipwait = false,
    .wren = false
  };
  
  nrfx_err_t err = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, jedec_id);
  if (err != NRFX_SUCCESS) {
    Serial.print("JEDEC read failed: 0x");
    Serial.println(err, HEX);
    return false;
  }
  
  Serial.print("JEDEC ID: ");
  for (int i = 0; i < 3; i++) {
    if (jedec_id[i] < 0x10) Serial.print("0");
    Serial.print(jedec_id[i], HEX);
    if (i < 2) Serial.print(" ");
  }
  Serial.println();
  
  // Check for valid ID
  if ((jedec_id[0] == 0x00 && jedec_id[1] == 0x00 && jedec_id[2] == 0x00) ||
      (jedec_id[0] == 0xFF && jedec_id[1] == 0xFF && jedec_id[2] == 0xFF)) {
    Serial.println("Invalid JEDEC ID - check connections");
    return false;
  }
  
  // Look for known chip
  for (uint32_t i = 0; i < qspiFlashChipCount; i++) {
    if (jedec_id[0] == qspiFlashChips[i].jedec_id[0] && 
        jedec_id[1] == qspiFlashChips[i].jedec_id[1] && 
        jedec_id[2] == qspiFlashChips[i].jedec_id[2]) {
      
      _chip = &qspiFlashChips[i];
      _total_size = _chip->total_size;
      _sector_size = _chip->sector_size;
      _page_size = _chip->page_size;
      
      Serial.print("Known chip: ");
      Serial.println(_chip->name);
      return true;
    }
  }
  
  Serial.println("Unknown chip - using defaults");
  return true;  // Continue with defaults
}

bool CustomLFS_QSPIFlash::qspiRead(uint32_t addr, void *buffer, uint32_t size)
{
  if (!_qspi_initialized || !buffer || size == 0) {
    return false;
  }
  
  // Simple blocking call - remove all async handling
  nrfx_err_t err = nrfx_qspi_read(buffer, size, addr);
  return (err == NRFX_SUCCESS);
}

bool CustomLFS_QSPIFlash::qspiWrite(uint32_t addr, const void *buffer, uint32_t size)
{
  if (!_qspi_initialized || !buffer || size == 0) {
    return false;
  }
  
  // Simple blocking call
  nrfx_err_t err = nrfx_qspi_write(buffer, size, addr);
  return (err == NRFX_SUCCESS);
}

bool CustomLFS_QSPIFlash::qspiErase(uint32_t addr, uint32_t size)
{
  if (!_qspi_initialized) {
    return false;
  }
  
  // Simple blocking call
  nrfx_err_t err = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, addr);
  return (err == NRFX_SUCCESS);
}

bool CustomLFS_QSPIFlash::qspiWaitReady(uint32_t timeout_ms)
{
  uint32_t start = millis();
  
  while ((millis() - start) < timeout_ms) {
    // Simple status check using custom instruction
    uint8_t status = 0xFF;
    nrf_qspi_cinstr_conf_t cfg = {
      .opcode = 0x05,  // Read status
      .length = NRF_QSPI_CINSTR_LEN_2B,
      .io2_level = true,
      .io3_level = true,
      .wipwait = false,
      .wren = false
    };
    
    if (nrfx_qspi_cinstr_xfer(&cfg, NULL, &status) == NRFX_SUCCESS) {
      if ((status & 0x01) == 0) {  // WIP bit clear
        return true;
      }
    }
    
    delay(1);
  }
  
  return false;
}

void CustomLFS_QSPIFlash::_configure_lfs()
{
  memset(&_lfs_config, 0, sizeof(_lfs_config));
  
  _lfs_config.context = this;
  
  // Callbacks
  _lfs_config.read = _qspi_read;
  _lfs_config.prog = _qspi_prog;
  _lfs_config.erase = _qspi_erase;
  _lfs_config.sync = _qspi_sync;
  
  // CRITICAL: Adjust these parameters for QSPI flash
  _lfs_config.read_size = 256;           // Match page size
  _lfs_config.prog_size = 256;           // Match page size
  _lfs_config.block_size = _sector_size; // 4096 bytes
  _lfs_config.block_count = _total_size / _sector_size;
  
  // Reduce lookahead for smaller flash
  _lfs_config.lookahead = 32;            // Smaller lookahead for 2MB flash. TODO research possible better values?
  
  // Allocate static buffers instead of dynamic (more reliable)
  static uint8_t read_buffer[256];
  static uint8_t prog_buffer[256];
  static uint8_t lookahead_buffer[32];
  
  _lfs_config.read_buffer = read_buffer;
  _lfs_config.prog_buffer = prog_buffer;
  _lfs_config.lookahead_buffer = lookahead_buffer;
  _lfs_config.file_buffer = NULL;        // Keep this dynamic
  
  // Update parent class members
  _flash_addr = 0;
  _flash_total_size = _total_size;
  _block_size = _sector_size;
  
  // Debug the configuration
  Serial.println("=== LittleFS Configuration ===");
  Serial.print("Block size: "); Serial.println(_lfs_config.block_size);
  Serial.print("Block count: "); Serial.println(_lfs_config.block_count);
  Serial.print("Read size: "); Serial.println(_lfs_config.read_size);
  Serial.print("Prog size: "); Serial.println(_lfs_config.prog_size);
  Serial.print("Lookahead: "); Serial.println(_lfs_config.lookahead);
  Serial.print("Total size: "); 
  Serial.print(_lfs_config.block_count * _lfs_config.block_size / 1024);
  Serial.println(" KB");
}

bool CustomLFS_QSPIFlash::testFlash()
{
  if (!_qspi_initialized) {
    return false;
  }
  
  Serial.println("Testing QSPI flash...");
  
  uint8_t testData[256];
  uint8_t readData[256];
  
  // Create test pattern
  for (int i = 0; i < 256; i++) {
    testData[i] = i ^ 0xAA;
  }
  
  // Use last sector
  uint32_t testAddr = _total_size - _sector_size;
  
  // Erase
  if (!qspiErase(testAddr)) {
    Serial.println("Erase failed");
    return false;
  }
  
  // Write
  if (!qspiWrite(testAddr, testData, sizeof(testData))) {
    Serial.println("Write failed");
    return false;
  }
  
  // Read
  if (!qspiRead(testAddr, readData, sizeof(readData))) {
    Serial.println("Read failed");
    return false;
  }
  
  // Compare
  for (int i = 0; i < 256; i++) {
    if (testData[i] != readData[i]) {
      Serial.print("Verify failed at ");
      Serial.println(i);
      return false;
    }
  }
  
  Serial.println("Flash test passed");
  return true;
}

bool CustomLFS_QSPIFlash::benchmark(uint32_t test_size)
{
  if (!_qspi_initialized) return false;
  
  Serial.println("Running benchmark...");
  Serial.print("Test size: ");
  Serial.println(test_size);
  
  // Simple timing test
  uint32_t start = millis();
  
  // Test reads
  uint8_t buffer[256];
  for (uint32_t addr = 0; addr < test_size; addr += sizeof(buffer)) {
    if (!qspiRead(addr, buffer, sizeof(buffer))) {
      Serial.println("Benchmark read failed");
      return false;
    }
  }
  
  uint32_t read_time = millis() - start;
  
  Serial.print("Read time: ");
  Serial.print(read_time);
  Serial.println(" ms");
  
  if (read_time > 0) {
    Serial.print("Read speed: ");
    Serial.print((test_size * 1000) / (read_time * 1024));
    Serial.println(" KB/s");
  }
  
  return true;
}

bool CustomLFS_QSPIFlash::lowLevelFormat()
{
  Serial.println("WARNING: This will erase all data!");
  Serial.println("Formatting flash...");
  
  uint32_t sectors = _total_size / _sector_size;
  for (uint32_t i = 0; i < sectors; i++) {
    if (!qspiErase(i * _sector_size)) {
      Serial.print("Format failed at sector ");
      Serial.println(i);
      return false;
    }
    
    if ((i % 32) == 0) {
      Serial.print("Progress: ");
      Serial.print((i * 100) / sectors);
      Serial.println("%");
    }
  }
  
  Serial.println("Format complete");
  return true;
}

void CustomLFS_QSPIFlash::debugQSPIState()
{
  Serial.println("=== QSPI Register Debug ===");
  Serial.print("ENABLE: 0x");
  Serial.println(NRF_QSPI->ENABLE, HEX);
  Serial.print("STATUS: 0x");
  Serial.println(NRF_QSPI->STATUS, HEX);
  Serial.print("IFCONFIG0: 0x");
  Serial.println(NRF_QSPI->IFCONFIG0, HEX);
  Serial.print("IFCONFIG1: 0x");
  Serial.println(NRF_QSPI->IFCONFIG1, HEX);
  
  // Check individual status bits
  bool ready = (NRF_QSPI->STATUS & QSPI_STATUS_READY_Msk) != 0;
  bool dpm = (NRF_QSPI->STATUS & QSPI_STATUS_DPM_Msk) != 0;
  bool sreg = (NRF_QSPI->STATUS & QSPI_STATUS_SREG_Msk) != 0;
  
  Serial.print("READY bit: "); Serial.println(ready);
  Serial.print("DPM bit: "); Serial.println(dpm);
  Serial.print("SREG bit: "); Serial.println(sreg);
}

bool CustomLFS_QSPIFlash::isQSPIReady()
{
  if (!_qspi_initialized) {
    return false;
  }
  
  // Check if QSPI is enabled and ready
  bool enabled = (NRF_QSPI->ENABLE == 1);
  bool ready = (NRF_QSPI->STATUS & QSPI_STATUS_READY_Msk) != 0;
  
  Serial.print("QSPI enabled: ");
  Serial.print(enabled);
  Serial.print(", ready: ");
  Serial.println(ready);
  
  return enabled && ready;
}


// Placeholder functions for later
bool CustomLFS_QSPIFlash::enableQuadMode() { return false; }  // Disable for now
bool CustomLFS_QSPIFlash::checkQuadMode() { return false; }
uint8_t CustomLFS_QSPIFlash::readStatus(uint8_t reg) { return 0xFF; }
bool CustomLFS_QSPIFlash::writeStatus(uint8_t val, uint8_t reg) { return false; }
bool CustomLFS_QSPIFlash::setClockFrequency(uint32_t freq) { return false; }
bool CustomLFS_QSPIFlash::enableMemoryMapping() { return _qspi_initialized; }
bool CustomLFS_QSPIFlash::disableMemoryMapping() { return true; }

#endif // NRF52840_XXAA