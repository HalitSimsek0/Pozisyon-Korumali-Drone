#define DEBUG_MODULE "SYS"
#include <stdint.h>
#include "FreeRTOS.h"
#include "cfassert.h"
#include "led.h"
#include "power_distribution.h"
#include "debug_cf.h"
#define MAGIC_ASSERT_INDICATOR 0x2f8a001f
enum snapshotType_e
{
  SnapshotTypeNone = 0,
  SnapshotTypeFile = 1,
  SnapshotTypeHardFault = 2,
  SnapshotTypeText = 3,
};
typedef struct SNAPSHOT_DATA {
  uint32_t magicNumber;
  enum snapshotType_e type;
  union {
    struct {
      const char* fileName;
      int line;
    } file;
    // Removed: HardFault struct - ARM Cortex-M register names (r0-r12, lr, pc, psr) 
    // ESP32 uses Xtensa architecture, not ARM Cortex-M. This code is STM32 legacy and unused.
    struct {
      unsigned int r0;  // Not applicable to ESP32/Xtensa
      unsigned int r1;
      unsigned int r2;
      unsigned int r3;
      unsigned int r12;
      unsigned int lr;   // Link register (ARM-specific)
      unsigned int pc;   // Program counter
      unsigned int psr;  // Program status register (ARM-specific)
    } hardfault;
    struct {
      const char* text;
    } text;
  };
} SNAPSHOT_DATA;
// Note: Snapshot is stored in RAM and will be lost on power cycle or full reset.
// For persistent crash logs on ESP32, consider using RTC memory, NVS, or flash storage.
SNAPSHOT_DATA snapshot = { 
  .magicNumber = 0,
  .type = SnapshotTypeNone,
};
void assertFail(char *exp, char *file, int line)
{
  portDISABLE_INTERRUPTS();
  storeAssertFileData(file, line);
  DEBUG_PRINTE("Assert failed %s:%d\n", file, line);
  ledClearAll();
  ledSet(ERR_LED1, 1);
  powerStop();
  while (1);
}
void storeAssertFileData(const char *file, int line)
{
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.type = SnapshotTypeFile;
  snapshot.file.fileName = file;
  snapshot.file.line = line;
}
// Note: This function is STM32/ARM Cortex-M specific and not applicable to ESP32/Xtensa.
// ESP32 uses different exception handling mechanisms. This function is never called in ESP32 builds.
void storeAssertHardfaultData(
    unsigned int r0,
    unsigned int r1,
    unsigned int r2,
    unsigned int r3,
    unsigned int r12,
    unsigned int lr,
    unsigned int pc,
    unsigned int psr)
{
  // ARM Cortex-M register names (r0-r12, lr, pc, psr) - not applicable to ESP32/Xtensa
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.type = SnapshotTypeHardFault;
  snapshot.hardfault.r0 = r0;
  snapshot.hardfault.r1 = r1;
  snapshot.hardfault.r2 = r2;
  snapshot.hardfault.r3 = r3;
  snapshot.hardfault.r12 = r12;
  snapshot.hardfault.lr = lr;
  snapshot.hardfault.pc = pc;
  snapshot.hardfault.psr = psr;
}
void storeAssertTextData(const char *text)
{
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.type = SnapshotTypeText;
  snapshot.text.text = text;
}
static void clearAssertData() {
  snapshot.type = SnapshotTypeNone;
}
void printAssertSnapshotData()
{
  if (MAGIC_ASSERT_INDICATOR == snapshot.magicNumber) {
    switch (snapshot.type) {
      case SnapshotTypeFile:
        DEBUG_PRINT_LOCAL("Assert failed at %s:%d\n", snapshot.file.fileName, snapshot.file.line);
        break;
      case SnapshotTypeHardFault:
        // Note: HardFault register printing is ARM Cortex-M specific (STM32 legacy)
        // ESP32 uses Xtensa architecture with different exception handling
        DEBUG_PRINT_LOCAL("Hardfault (STM32 legacy - not applicable to ESP32). r0: %X, r1: %X, r2: %X, r3: %X, r12: %X, lr: %X, pc: %X, psr: %X\n",
          snapshot.hardfault.r0,
          snapshot.hardfault.r1,
          snapshot.hardfault.r2,
          snapshot.hardfault.r3,
          snapshot.hardfault.r12,
          snapshot.hardfault.lr,
          snapshot.hardfault.pc,
          snapshot.hardfault.psr);
        break;
      case SnapshotTypeText:
        DEBUG_PRINT_LOCAL("Assert failed: %s\n", snapshot.text.text);
        break;
      default:
        DEBUG_PRINT_LOCAL("Assert failed, but unknown type\n");
        break;
    }
  } else {
    DEBUG_PRINT_LOCAL( "No assert information found\n");
  }
}
static bool isAssertRegistered() {
  return (MAGIC_ASSERT_INDICATOR == snapshot.magicNumber) && (snapshot.type != SnapshotTypeNone);
}
bool cfAssertNormalStartTest(void) {
  bool wasNormalStart = true;
	if (isAssertRegistered()) {
		wasNormalStart = false;
		DEBUG_PRINT("The system resumed after a failed assert [WARNING]\n");
		printAssertSnapshotData();
    clearAssertData();
	}
	return wasNormalStart;
}
