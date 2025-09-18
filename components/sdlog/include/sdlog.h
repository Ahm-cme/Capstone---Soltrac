#pragma once
#include <stdarg.h>
#include <stdbool.h>

/*
    ┌───────────────────────────────────────────────────────────────────────┐
    │ SD Card Logging System                                                │
    │ - Simple append-only logger for tracking data and system events      │
    │ - Uses ESP32's VFS + FAT filesystem over SPI mode SD card            │
    │ - Designed for solar tracker: CSV data + human-readable logs         │
    └───────────────────────────────────────────────────────────────────────┘

    Design:
    - Keep it simple: append-only files, no fancy indexing or rotation.
    - Robustness over performance: each write is a complete open/write/close cycle
      to ensure data hits the card even if we crash or lose power unexpectedly.
    - Two file types: structured CSV for data analysis, plain text logs for debugging.
    - Minimal RAM usage: we don't buffer writes, everything goes straight to the card.

    File structure created:
    /sdcard/soltrac.log     - Human-readable system events, errors, state changes
    /sdcard/soltrac.csv     - Structured tracking data for analysis/plotting
    /sdcard/debug.log       - Optional debug trace (if you enable verbose logging)

    Hardware considerations:
    - SD cards have limited write cycles (~10K-100K depending on type)
    - We write once per tracking loop (every 5-15 min) so ~35-100 writes/day
    - At this rate, even a cheap SD card should last 1-3 years minimum
    - Use industrial/automotive SD cards for harsh outdoor environments
    - Consider write-protection: GPS coords are valuable if the tracker is stolen!

    SPI mode limitations:
    - Slower than native SDIO mode, but more compatible and uses fewer pins
    - ESP32-CAM boards typically wire SD in SPI mode on pins 2,13,14,15
    - Pin 2 is a boot strap pin - ensure SD card isn't inserted during flashing
    - Pin 15 is also a strap pin - avoid holding it low during boot

    Filesystem notes:
    - We use FAT32 which is widely compatible but has file size limits (4GB)
    - For long-term deployment, consider log rotation or periodic archival
    - SD card failure is a common failure mode - always have fallback logging to NVS

    Power considerations:
    - SD card can draw 50-200mA during writes (brief but significant)
    - Ensure your power supply can handle the inrush current
    - Consider adding a small capacitor near the SD card for supply stability

    Threading safety:
    - These functions are NOT thread-safe by design (keep it simple)
    - Call only from the main tracking task or use external mutex if needed
    - Each function opens/closes files independently to avoid state corruption
*/

typedef struct {
    // SPI pins for SD (ESP32-CAM default SPI wiring)
    int mosi;  // 15 - Master Out, Slave In (ESP32 → SD card)
    int miso;  // 2  - Master In, Slave Out (SD card → ESP32) [BOOT STRAP PIN!]
    int sclk;  // 14 - Serial Clock (ESP32 → SD card)
    int cs;    // 13 - Chip Select (ESP32 → SD card, active low)
} sdlog_cfg_t;

/*
    Initialize SD card and mount FAT filesystem.
    
    This function:
    1. Configures SPI bus with the provided pin assignments
    2. Initializes SDSPI host driver with default DMA settings  
    3. Attempts to mount FAT filesystem at /sdcard
    4. Logs card information (manufacturer, capacity, etc.)
    
    Configuration details:
    - SPI speed: defaults to 20 MHz (safe for most cards)
    - DMA channel: SDSPI_DEFAULT_DMA (usually DMA channel 2)
    - Max transfer size: 4000 bytes (good balance of speed vs RAM)
    - File allocation: 16KB clusters (good for large sequential writes)
    - Max open files: 5 (enough for log + csv + temp files)
    
    Error handling:
    - Returns false on any failure (SPI config, card detect, mount, etc.)
    - Logs detailed error messages to help diagnose hardware issues
    - Safe to call multiple times (will skip if already mounted)
    
    Common failure modes:
    - No SD card inserted: mount will fail with "no medium" error
    - Corrupted filesystem: mount will fail, may need reformatting
    - Bad SPI wiring: SPI init will fail or card won't respond
    - Power supply issues: intermittent failures or corruption
    
    Parameters:
    - cfg: pin assignments and SPI configuration
    
    Returns:
    - true: SD card mounted successfully, ready for logging
    - false: mount failed, check logs for details
*/
bool sdlog_init(const sdlog_cfg_t *cfg);

/*
    Write a timestamped log entry to /sdcard/soltrac.log
    
    This is the "human readable" log for system events, errors, and state changes.
    Each entry gets a timestamp and newline automatically added.
    
    Format: [timestamp] message
    
    Usage examples:
    - sdlog_printf("System startup complete");
    - sdlog_printf("GPS fix lost, using last known position");
    - sdlog_printf("Deep sleep until %ld", (long)wake_time);
    - sdlog_printf("Motor move: AZ %.1f° → %.1f°", current, target);
    
    Thread safety: NOT thread-safe (opens/writes/closes on each call)
    File handling: append mode, fflush() forces immediate write to card
    Error behavior: silent failure if SD not mounted or file can't open
    
    Parameters:
    - fmt: printf-style format string
    - ...: format arguments
*/
void sdlog_printf(const char *fmt, ...);

/*
    Create CSV header row if the file doesn't exist yet.
    
    This ensures our CSV files have proper column headers for analysis tools.
    The header matches the format expected by sdlog_write_csv().
    
    Standard CSV columns:
    - unix_ts: Unix timestamp (seconds since 1970)
    - lat, lon: GPS coordinates (decimal degrees)
    - fix: GPS fix type (0=none, 2=2D, 3=3D)
    - sats: number of satellites used in solution
    - az_target, el_target: commanded panel angles (degrees)
    - az_cur, el_cur: current panel angles (degrees, estimated)
    - moves_today: count of moves since midnight
    - total_moves: lifetime move counter
    - batt_v: battery voltage (if available)
    - notes: text field for state/event info
    
    Behavior:
    - If file exists, does nothing (preserves existing data)
    - If file doesn't exist, creates it and writes header row
    - Silent failure if SD not mounted or file can't be created
    
    Parameters:
    - path: full path to CSV file (typically "/sdcard/soltrac.csv")
*/
void sdlog_write_csv_header_if_new(const char *path);

/*
    Append a data row to the specified CSV file.
    
    Use this for structured tracking data that you'll analyze later.
    The format string should match the header created by sdlog_write_csv_header_if_new().
    
    Typical usage:
    sdlog_write_csv("/sdcard/soltrac.csv", 
                    "%ld,%.7f,%.7f,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%.2f,%s",
                    time(NULL), gps.lat, gps.lon, gps.fix_type, gps.num_sats,
                    sun_az, sun_el, panel_az, panel_el, moves_today, total_moves,
                    battery_volts, "TRACKING");
    
    CSV best practices:
    - Use consistent decimal places for angles (%.2f for degrees)
    - Use high precision for GPS coords (%.7f ≈ 1cm resolution)
    - Quote text fields that might contain commas
    - Avoid newlines in text fields (use spaces or underscores)
    - Use NaN or empty string for missing numeric data
    
    Performance notes:
    - Each call opens the file, writes one line, and closes it
    - This is slower than batch writes but much more crash-safe
    - For high-frequency logging, consider batching or using a RAM buffer
    
    Parameters:
    - path: full path to CSV file
    - fmt: printf-style format string for the CSV row
    - ...: format arguments matching the format string
*/
void sdlog_write_csv(const char *path, const char *fmt, ...);

